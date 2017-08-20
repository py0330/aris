#include "test_dynamic_model.h"
#include <iostream>
#include <aris.h>

#include<type_traits>

using namespace aris::dynamic;

const char xml_file_3R[] =
"<?xml version=\"1.0\" encoding=\"utf-8\"?>"
"<root>"
"    <model>"
"        <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"        <variable_pool type=\"VariablePoolElement\" default_child_type=\"Matrix\">"
"            <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"            <Mot_friction type=\"MatrixVariable\">{0, 0, 0}</Mot_friction>"
"        </variable_pool>"
"        <part_pool type=\"PartPoolElement\" default_child_type=\"Part\">"
"            <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <origin pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </ground>"
"            <part1 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part1.x_t\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <r1i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r2j pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t\"/>"
"                </geometry_pool>"
"            </part1>"
"            <part2 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,0,0,PI/2,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part2.x_t\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <r2i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r3j pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part2.x_t\"/>"
"                </geometry_pool>"
"            </part2>"
"            <part3 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0.2,0.5,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <r3i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <ee pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part3.x_t\"/>"
"                </geometry_pool>"
"            </part3>"
"            <part4 active=\"false\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0.2,0.5,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part3.x_t\"/>"
"                </geometry_pool>"
"            </part4>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolElement\">"
"            <r1 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\"/>"
"            <r2 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\"/>"
"            <r3 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"            <r4 active=\"false\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolElement\" default_child_type=\"Motion\">"
"            <m1 active=\"true\" slave_id=\"0\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"            <m2 active=\"true\" slave_id=\"1\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"            <m3 active=\"true\" slave_id=\"2\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolElement\" default_child_type=\"GeneralMotion\">"
"            <ee_mot type=\"GeneralMotion\" active=\"true\" prt_m=\"part3\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"        </general_motion_pool>"
"        <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"            <gs type=\"LltGroundDividedSolver\"/>"
"            <ps type=\"LltPartDividedSolver\"/>"
"            <ds type=\"DiagSolver\"/>"
"        </solver_pool>"
"    </model>"
"</root>";

const char xml_file_6R[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<Root>"
"    <model>"
"        <environment type=\"Environment\" gravity=\"{0 , -9.8 , 0 , 0 , 0 , 0}\"/>"
"        <variable_pool type=\"VariablePoolElement\"/>"
"        <part_pool type=\"PartPoolElement\">"
"            <ground type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <R0j type=\"Marker\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\"/>"
"                    <origin type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                </marker_pool>"
"            </ground>"
"            <part0 type=\"Part\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <R0i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R1j type=\"Marker\" active=\"true\" pe=\"{0.127581045131803 , -0.255551891814505 , 0.743785121868611 , 1.39801111882379 , 0.92874853281329 , 2.99910369947683}\"/>"
"                </marker_pool>"
"            </part0>"
"            <part1 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.83142459280181 , 1.5646920289751 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <R1i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R2j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 6.21426444075139 , 1.09203109440023 , 1.87858662805362}\"/>"
"                </marker_pool>"
"            </part1>"
"            <part2 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322217}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <R2i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R3j type=\"Marker\" active=\"true\" pe=\"{0 , -0.996734365258792 , 0.0807502638519182 , 1.5707963267949 , 7.66425882286152e-17 , 4.71238898038469}\"/>"
"                </marker_pool>"
"            </part2>"
"            <part3 type=\"Part\" active=\"true\" pe=\"{1.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <R3i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R4j type=\"Marker\" active=\"true\" pe=\"{1.53280055883457 , 0.0630122434114775 , 0.777786541421676 , 4.71238898038469 , 1.11022302462516e-16 , 1.5707963267949}\"/>"
"                </marker_pool>"
"            </part3>"
"            <part4 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <R4i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R5j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 0.299854573312955 , 1.27790273658208 , 3.56732345865295}\"/>"
"                </marker_pool>"
"            </part4>"
"            <part5 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 1.928105009803 , 1.5084212451233 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <R5i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <end_effector type=\"Marker\" active=\"true\" pe=\"{1.6840663068739 , 0.615533362716626 , -2.33680109411025 , -0 , 1.5084212451233 , 1.2134876437868}\"/>"
"                </marker_pool>"
"            </part5>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolElement\">"
"            <R0 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\"/>"
"            <R1 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\"/>"
"            <R2 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\"/>"
"            <R3 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\"/>"
"            <R4 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\"/>"
"            <R5 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolElement\">"
"            <M0 type=\"Motion\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M1 type=\"Motion\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M2 type=\"Motion\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M3 type=\"Motion\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M4 type=\"Motion\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M5 type=\"Motion\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolElement\">"
"            <ee_mot type=\"GeneralMotion\" active=\"true\" prt_m=\"part5\" prt_n=\"ground\" mak_i=\"end_effector\" mak_j=\"origin\"/>"
"        </general_motion_pool>"
"        <force_pool type=\"ForcePoolElement\"/>"
"        <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"            <gs type=\"LltGroundDividedSolver\"/>"
"            <ps type=\"LltPartDividedSolver\"/>"
"            <ds type=\"DiagSolver\"/>"
"        </solver_pool>"
"    </model>"
"</Root>";

const char xml_file_stewart[] =
"<?xml version=\"1.0\" encoding=\"utf-8\"?>"
"<root>"
"    <model>"
"        <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"        <variable_pool type=\"VariablePoolElement\" default_child_type=\"Matrix\">"
"            <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"            <Mot_friction type=\"MatrixVariable\">{0, 0, 0}</Mot_friction>"
"        </variable_pool>"
"        <part_pool type=\"PartPoolElement\" default_child_type=\"Part\">"
"            <p1a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.999999999999974 , 1.22522177619812e-16 , -9.28869564848867e-18 , 6.38378239159465e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <u1i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p1j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"                </geometry_pool>"
"            </p1a>"
"            <p1b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.0711481425892889 , 1.49999999999963 , 0.912443796234424 , 8.04911692853238e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <p1i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"                </geometry_pool>"
"            </p1b>"
"            <p2a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.999999999999995 , 1.22524189323061e-16 , -9.2876368573046e-18 , 5.55111512312578e-17 , 0.721024145526766 , 0.308719565228027}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <u2i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p2j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"                </geometry_pool>"
"            </p2a>"
"            <p2b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.363127053316677 , 1.49999999999988 , 1.31832224563822 , 6.28318530717959 , 0.721024145526766 , 0.308719565228028}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <p2i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s2j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"                </geometry_pool>"
"            </p2b>"
"            <p3a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1.24902578429613e-16 , 3.6066466064807e-14 , 1.73199999999999 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <u3i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p3j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"                </geometry_pool>"
"            </p3a>"
"            <p3b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.363127053316337 , 1.49999999999935 , 1.31832224563851 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <p3i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s3j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"                </geometry_pool>"
"            </p3b>"
"            <p4a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1.24898250620648e-16 , 1.52855080404276e-14 , 1.732 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <u4i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p4j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"                </geometry_pool>"
"            </p4a>"
"            <p4b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{-0.134375029322252 , 1.49999999999964 , 1.36823895396183 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <p4i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s4j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"                </geometry_pool>"
"            </p4b>"
"            <p5a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{-0.999999999999993 , -1.0082029353865e-16 , 4.19175032725778e-17 , 6.28318530717959 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <u5i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p5j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"                </geometry_pool>"
"            </p5a>"
"            <p5b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{-0.134375029322406 , 1.49999999999987 , 1.36823895396163 , 2.77555756156289e-17 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <p5i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s5j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"                </geometry_pool>"
"            </p5b>"
"            <p6a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{-0.999999999999969 , -1.00821934664985e-16 , 4.19165900651815e-17 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <u6i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p6j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"                </geometry_pool>"
"            </p6a>"
"            <p6b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.0711481425888235 , 1.49999999999959 , 0.912443796234401 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <p6i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s6j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"                </geometry_pool>"
"            </p6b>"
"            <up active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.1 , 1.5 , 1.2 , 1.5707963267949 , 0.1 , 4.71238898038469}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <ee pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <s1i pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                    <s2i pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                    <s3i pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                    <s4i pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                    <s5i pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                    <s6i pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\up.xmt_txt\"/>"
"                </geometry_pool>"
"            </up>"
"            <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <origin pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <u1o pe=\"{ 1,0,0,0,0,0 }\"/>"
"                    <u2o pe=\"{ 1,0,0,0,0,0 }\"/>"
"                    <u3o pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                    <u4o pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                    <u5o pe=\"{ -1,0,0,0,0,0 }\"/>"
"                    <u6o pe=\"{ -1,0,0,0,0,0 }\"/>"
"                    <u1j pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                    <u2j pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                    <u3j pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                    <u4j pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                    <u5j pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                    <u6j pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                </marker_pool>"
"            </ground>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolElement\">"
"            <u1 active=\"true\" type=\"UniversalJoint\" prt_m=\"p1a\" prt_n=\"ground\" mak_i=\"u1i\" mak_j=\"u1j\"/>"
"            <p1 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\"/>"
"            <s1 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p1b\" mak_i=\"s1i\" mak_j=\"s1j\"/>"
"            <u2 active=\"true\" type=\"UniversalJoint\" prt_m=\"p2a\" prt_n=\"ground\" mak_i=\"u2i\" mak_j=\"u2j\"/>"
"            <p2 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\"/>"
"            <s2 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p2b\" mak_i=\"s2i\" mak_j=\"s2j\"/>"
"            <u3 active=\"true\" type=\"UniversalJoint\" prt_m=\"p3a\" prt_n=\"ground\" mak_i=\"u3i\" mak_j=\"u3j\"/>"
"            <p3 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\"/>"
"            <s3 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p3b\" mak_i=\"s3i\" mak_j=\"s3j\"/>"
"            <u4 active=\"true\" type=\"UniversalJoint\" prt_m=\"p4a\" prt_n=\"ground\" mak_i=\"u4i\" mak_j=\"u4j\"/>"
"            <p4 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\"/>"
"            <s4 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p4b\" mak_i=\"s4i\" mak_j=\"s4j\"/>"
"            <u5 active=\"true\" type=\"UniversalJoint\" prt_m=\"p5a\" prt_n=\"ground\" mak_i=\"u5i\" mak_j=\"u5j\"/>"
"            <p5 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\"/>"
"            <s5 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p5b\" mak_i=\"s5i\" mak_j=\"s5j\"/>"
"            <u6 active=\"true\" type=\"UniversalJoint\" prt_m=\"p6a\" prt_n=\"ground\" mak_i=\"u6i\" mak_j=\"u6j\"/>"
"            <p6 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\"/>"
"            <s6 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p6b\" mak_i=\"s6i\" mak_j=\"s6j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolElement\" default_child_type=\"Motion\">"
"            <m1 active=\"true\" slave_id=\"0\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m2 active=\"true\" slave_id=\"9\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m3 active=\"true\" slave_id=\"8\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m4 active=\"true\" slave_id=\"3\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m5 active=\"true\" slave_id=\"7\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m6 active=\"true\" slave_id=\"5\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolElement\" default_child_type=\"GeneralMotion\">"
"            <ee_mot type=\"GeneralMotion\" active=\"false\" prt_m=\"up\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"        </general_motion_pool>"
"        <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"            <gs type=\"LltGroundDividedSolver\"/>"
"            <ps type=\"LltPartDividedSolver\"/>"
"            <ds type=\"DiagSolver\"/>"
"        </solver_pool>"
"    </model>"
"</root>";

void test_part()
{
	aris::dynamic::Model model;

	const double im[36]{ 12.3,0,0,0,0.356, -0.2228,
		0,12.3,0,-0.356,0,0.1,
		0,0,12.3,0.2228, -0.1,0,
		0, -0.356,0.2228,5.8,0.85,0.75,
		0.356,0, -0.1,0.85,6.4,0.98,
		-0.2228,0.1,0,0.75,0.98,3.9 };

	auto &p = model.partPool().add<Part>("test_part",im);
	auto &r = model.partPool().add<Part>("relative_part",im);

	const double pp[3] = { 0.1, 0.2, 0.3 };
	const double re313[3] = { 0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double re321[3] = { 2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double rq[4] = { 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double rm[9] = { -0.22, -0.975499782797526,   0.000416847668728071,
		0.175499782797526, -0.04, -0.983666521865018,
		0.959583152331272, -0.216333478134982,   0.18 };
	const double pe313[6] = { 0.1, 0.2, 0.3,0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double pe321[6] = { 0.1, 0.2, 0.3,2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double pq[7] = { 0.1, 0.2, 0.3,0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18,0.3,
		0,0,0,1 };

	const double vp[3] = { 0.307558670154491,   1.2433000508379, -1.04895965543501 };
	const double we313[3] = { -0.644213536852877, -0.245050866834802, -1.27836042009784 };
	const double we321[3] = { -4.19969388864156, -0.83045134600268,   3.46543753721832 };
	const double wq[4] = { 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
	const double wm[9] = { 1.36, -0.30698536874045, -0.633709981238717,
		0.426985368740452,   0.8,   0.0436487757967661,
		0.233709981238715,   1.23635122420323,   0.24 , };
	const double ve313[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-0.644213536852877, -0.245050866834802, -1.27836042009784 };
	const double ve321[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-4.19969388864156, -0.83045134600268,   3.46543753721832 };
	const double vq[7] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
	const double vm[16] = { 1.36, -0.30698536874045, -0.633709981238717,0.307558670154491,
		0.426985368740452,   0.8,   0.0436487757967661,1.2433000508379,
		0.233709981238715,   1.23635122420323,   0.24 , -1.04895965543501,
		0,0,0,0 };
	const double wa[3] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	const double va[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	const double vs[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };

	const double ap[3] = { 2.2628985000154, -0.843606386309081, -0.248846478459814 };
	const double xe313[3] = { 1.51734920338156,   1.71538128045296,   1.3693196878275 };
	const double xe321[3] = { -15.6049676192293,   4.50445705187534,   16.9352080725126 };
	const double xq[4] = { -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
	const double xm[9] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,
		-2.32024759895695, -0.653600000000002,   0.450521351741563,
		-1.92944395313478, -1.05972135174157, -0.103200000000001 };
	const double ae313[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 1.51734920338156,   1.71538128045296,   1.3693196878275 };
	const double ae321[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -15.6049676192293,   4.50445705187534,   16.9352080725126 };
	const double aq[7] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
	const double am[16] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,2.2628985000154,
		-2.32024759895695, -0.653600000000002,   0.450521351741563,-0.843606386309081,
		-1.92944395313478, -1.05972135174157, -0.103200000000001,-0.248846478459814,
		0,0,0,0 };
	const double xa[3] = { 0.904633672502324, -1.24440604199266,   1.45568007018557 };
	const double aa[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 0.904633672502324, -1.24440604199266,   1.45568007018557 };
	const double as[6] = { 3.15925342342501, -0.192390604845803,   0.136512424183815,   0.904633672502324, -1.24440604199266,   1.45568007018557 };



	const double relative_vs[16] = { 0.12, -0.35, 0.26, 0.58, 0.36, -0.135 };
	const double relative_as[16] = { 0.14, 1.35, -0.35, -0.56, -0.34, 0.14 };
	const double relative_pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071,   0.1,
		0.175499782797526, -0.04, -0.983666521865018,   0.2,
		0.959583152331272, -0.216333478134982,   0.18,   0.3,
		0,   0,   0,   1 };

	const double to_pm[4][4] = { -0.1224,   0.253539765421328,   0.959549804517774, - 0.116974902258887,
		- 0.989539765421329,   0.0432, - 0.137640156385781, - 0.0855499782797527,
		- 0.0763498045177736, - 0.966359843614219,   0.2456,   0.406691619606131,
		0,   0,   0,   1 };
	const double to_vs[6] = { -1.4159950169102,   0.131290065274018, - 0.0927140779913885, - 0.593141011344806,   1.12682984222913, - 0.799025264483263 };
	const double to_as[6] = { 0.25059773457297,   2.12918260428844,   2.93584830296579,   0.319978146055887, -1.01985580694063,   2.40639240365168 };

	const double error = 1e-10;

	double result[42], result2[16], result3[16];

	r.setPm(relative_pm);
	r.setVs(relative_vs);
	r.setAs(relative_as);

	p.setPp(pp);
	if (!(s_is_equal(3, &pm[3], 4, &p.pm()[0][3], 4, error)))std::cout << "\"part:setPp\" failed" << std::endl;

	p.setPp(r, pp);
	if (!(s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error)))std::cout << "\"part:setPp relative\" failed" << std::endl;

	double pp_tem[3]{ 0.4,0.5,0.6 }; p.setPp(pp_tem);
	p.setRe(re313);
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error)&& s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRe\" failed" << std::endl;

	p.setRe(r, re313);
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRe relative\" failed" << std::endl;

	p.setRe(re321, "321");
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRe 321\" failed" << std::endl;

	p.setRe(r, re321, "321");
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRe 321 relative\" failed" << std::endl;

	p.setRq(rq);
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRq\" failed" << std::endl;

	p.setRq(r, rq);
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRq relative\" failed" << std::endl;

	p.setRm(rm);
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRm\" failed" << std::endl;

	p.setRm(r, rm);
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRm relative\" failed" << std::endl;

	p.setRm(pm, 4);
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRm with ld\" failed" << std::endl;

	p.setRm(r, pm, 4);
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRm with ld relative\" failed" << std::endl;
	
	p.setPe(pe313);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPe\" failed" << std::endl;

	p.setPe(r, pe313);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPe relative\" failed" << std::endl;

	p.setPe(pe321, "321");
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPe\" failed" << std::endl;

	p.setPe(r, pe321, "321");
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPe 321 relative\" failed" << std::endl;

	p.setPq(pq);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPq\" failed" << std::endl;

	p.setPq(r, pq);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPq relative\" failed" << std::endl;

	p.setPm(pm);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPm\" failed" << std::endl;

	p.setPm(r, pm);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPm relative\" failed" << std::endl;

	p.setWa(wa, rm);
	p.setVp(vp, pp);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVp\" failed" << std::endl;

	p.setWa(r, wa, rm);
	p.setVp(r, vp, pp);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVp relative\" failed" << std::endl;

	p.setWa(wa, rm);
	p.setPp(pp);
	p.setVp(vp);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVp\" failed" << std::endl;

	p.setWa(r, wa, rm);
	p.setPp(r, pp);
	p.setVp(r, vp);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVp relative\" failed" << std::endl;

	p.setVp(vp, pp);
	p.setWe(we313, re313);
	p.getVp(result);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error)&& s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWe\" failed" << std::endl;

	p.setWe(r, we313, re313);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWe relative\" failed" << std::endl;

	p.setRe(re313);
	p.setWe(we313);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWe\" failed" << std::endl;

	p.setRe(r, re313);
	p.setWe(r, we313);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWe relative\" failed" << std::endl;

	p.setWe(we321, re321, "321");
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWe 321\" failed" << std::endl;

	p.setWe(r, we321, re321, "321");
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWe 321 relative\" failed" << std::endl;

	p.setRe(re321, "321");
	p.setWe(we321, nullptr, "321");
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWe 321\" failed" << std::endl;

	p.setRe(r, re321, "321");
	p.setWe(r, we321, nullptr, "321");
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWe 321 relative\" failed" << std::endl;

	p.setWq(wq, rq);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWq\" failed" << std::endl;

	p.setWq(r, wq, rq);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWq relative\" failed" << std::endl;

	p.setRq(rq);
	p.setWq(wq);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWq\" failed" << std::endl;

	p.setRq(r, rq);
	p.setWq(r, wq);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWq relative\" failed" << std::endl;

	p.setWm(wm, rm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm\" failed" << std::endl;

	p.setWm(r, wm, rm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm relative\" failed" << std::endl;

	p.setRm(rm);
	p.setWm(wm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm\" failed" << std::endl;

	p.setRm(r, rm);
	p.setWm(r, wm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm relative\" failed" << std::endl;

	p.setWm(vm, pm, 4, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm with ld\" failed" << std::endl;

	p.setWm(r, vm, pm, 4, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm with ld relative\" failed" << std::endl;

	p.setRm(pm, 4);
	p.setWm(vm, nullptr, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm with ld\" failed" << std::endl;

	p.setRm(r, pm, 4);
	p.setWm(r, vm, nullptr, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm with ld relative\" failed" << std::endl;

	p.setWa(wa, rm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm\" failed" << std::endl;

	p.setWa(r, wa, rm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm relative\" failed" << std::endl;

	p.setRm(rm);
	p.setWa(wa);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm\" failed" << std::endl;

	p.setRm(r, rm);
	p.setWa(r, wa);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm relative\" failed" << std::endl;

	p.setWa(wa, pm, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWa with ld\" failed" << std::endl;

	p.setWa(r, wa, pm, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWa with ld relative\" failed" << std::endl;

	p.setRm(pm, 4);
	p.setWa(wa);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWa with ld\" failed" << std::endl;

	p.setRm(r, pm, 4);
	p.setWa(r, wa);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWa with ld relative\" failed" << std::endl;

	p.setVe(ve313, pe313);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVe\" failed" << std::endl;

	p.setVe(r, ve313, pe313);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVe relative\" failed" << std::endl;

	p.setPe(pe313);
	p.setVe(ve313);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVe\" failed" << std::endl;

	p.setPe(r, pe313);
	p.setVe(r, ve313);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVe relative\" failed" << std::endl;

	p.setVe(ve321, pe321, "321");
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVe 321\" failed" << std::endl;

	p.setVe(r, ve321, pe321, "321");
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVe 321 relative\" failed" << std::endl;

	p.setPe(pe321, "321");
	p.setVe(ve321, nullptr, "321");
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVe 321\" failed" << std::endl;

	p.setPe(r, pe321, "321");
	p.setVe(r, ve321, nullptr, "321");
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVe 321 relative\" failed" << std::endl;

	p.setVq(vq, pq);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVq\" failed" << std::endl;

	p.setVq(r, vq, pq);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVq relative\" failed" << std::endl;

	p.setPq(pq);
	p.setVq(vq);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVq\" failed" << std::endl;

	p.setPq(r, pq);
	p.setVq(r, vq);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVq relative\" failed" << std::endl;

	p.setVm(vm, pm);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVm\" failed" << std::endl;

	p.setVm(r, vm, pm);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVm relative\" failed" << std::endl;

	p.setPm(pm);
	p.setVm(vm);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVm\" failed" << std::endl;

	p.setPm(r, pm);
	p.setVm(r, vm);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVm relative\" failed" << std::endl;

	p.setVa(va, pp);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(3, pp, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setVa\" failed" << std::endl;

	p.setVa(r, va, pp);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error)))std::cout << "\"part:setVa relative\" failed" << std::endl;

	p.setPp(pp);
	p.setVa(va);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(3, pp, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setVa\" failed" << std::endl;

	p.setPp(r, pp);
	p.setVa(r, va);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error)))std::cout << "\"part:setVa relative\" failed" << std::endl;

	p.setVs(vs, pm);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVs\" failed" << std::endl;

	p.setVs(r, vs, pm);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVs relative\" failed" << std::endl;

	p.setPm(pm);
	p.setVs(vs);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVs\" failed" << std::endl;

	p.setPm(r, pm);
	p.setVs(r, vs);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVs relative\" failed" << std::endl;


	p.setXa(xa, wa, rm);
	p.setAp(ap, vp, pp);
	if (!(s_is_equal(6, as, p.as(), error) && s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setAp\" failed" << std::endl;

	p.setXa(r, xa, wa, rm);
	p.setAp(r, ap, vp, pp);
	if (!(s_is_equal(6, to_as, p.as(), error) && s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setAp relative\" failed" << std::endl;

	p.setXa(xa, wa, rm);
	p.setVp(vp, pp);
	p.setAp(ap);
	if (!(s_is_equal(6, as, p.as(), error) && s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setAp\" failed" << std::endl;

	p.setXa(r, xa, wa, rm);
	p.setVp(r, vp, pp);
	p.setAp(r, ap);
	if (!(s_is_equal(6, to_as, p.as(), error) && s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setAp relative\" failed" << std::endl;

	p.setAp(ap, vp, pp);

	p.setXe(xe313, we313, re313);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXe 313\" failed" << std::endl;

	p.setXe(r, xe313, we313, re313);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXe 313 relative\" failed" << std::endl;

	p.setWe(we313, re313);
	p.setXe(xe313, nullptr, nullptr);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXe 313\" failed" << std::endl;

	p.setWe(r, we313, re313);
	p.setXe(r, xe313, nullptr, nullptr);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXe 313 relative\" failed" << std::endl;

	p.setXe(xe321, we321, re321, "321");
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXe 321\" failed" << std::endl;

	p.setXe(r, xe321, we321, re321, "321");
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXe 321 relative\" failed" << std::endl;

	p.setWe(we321, re321, "321");
	p.setXe(xe321, nullptr, nullptr, "321");
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXe 321\" failed" << std::endl;

	p.setWe(r, we321, re321, "321");
	p.setXe(r, xe321, nullptr, nullptr, "321");
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXe 321 relative\" failed" << std::endl;

	p.setXq(xq, wq, rq);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXq\" failed" << std::endl;

	p.setXq(r, xq, wq, rq);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXq relative\" failed" << std::endl;

	p.setWq(wq, rq);
	p.setXq(xq);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXq\" failed" << std::endl;

	p.setWq(r, wq, rq);
	p.setXq(r, xq);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXq relative\" failed" << std::endl;

	p.setXm(xm, wm, rm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXm\" failed" << std::endl;

	p.setXm(r, xm, wm, rm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXm relative\" failed" << std::endl;

	p.setWm(wm, rm);
	p.setXm(xm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXm\" failed" << std::endl;

	p.setWm(r, wm, rm);
	p.setXm(r, xm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXm relative\" failed" << std::endl;

	p.setXa(xa, wa, rm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXa\" failed" << std::endl;

	p.setXa(r, xa, wa, rm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXa relative\" failed" << std::endl;

	p.setWa(wa, rm);
	p.setXa(xa);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXa\" failed" << std::endl;

	p.setWa(r, wa, rm);
	p.setXa(r, xa);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXa relative\" failed" << std::endl;



	p.setAe(ae313, ve313, pe313);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAe\" failed" << std::endl;

	p.setAe(r, ae313, ve313, pe313);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAe relative\" failed" << std::endl;

	p.setVe(ve313, pe313);
	p.setAe(ae313);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAe\" failed" << std::endl;

	p.setVe(r, ve313, pe313);
	p.setAe(r, ae313);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAe relative\" failed" << std::endl;

	p.setAe(ae321, ve321, pe321, "321");
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAe 321\" failed" << std::endl;

	p.setAe(r, ae321, ve321, pe321, "321");
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAe 321 relative\" failed" << std::endl;

	p.setVe(ve321, pe321, "321");
	p.setAe(ae321, nullptr, nullptr, "321");
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAe 321\" failed" << std::endl;

	p.setVe(r, ve321, pe321, "321");
	p.setAe(r, ae321, nullptr, nullptr, "321");
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAe 321 relative\" failed" << std::endl;

	p.setAq(aq, vq, pq);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAq\" failed" << std::endl;

	p.setAq(r, aq, vq, pq);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAq relative\" failed" << std::endl;

	p.setVq(vq, pq);
	p.setAq(aq);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAq\" failed" << std::endl;

	p.setVq(r, vq, pq);
	p.setAq(r, aq);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAq relative\" failed" << std::endl;

	p.setAm(am, vm, pm);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAm\" failed" << std::endl;

	p.setAm(r, am, vm, pm);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAm relative\" failed" << std::endl;

	p.setVm(vm, pm);
	p.setAm(am);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAm\" failed" << std::endl;

	p.setVm(r, vm, pm);
	p.setAm(r, am);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAm relative\" failed" << std::endl;

	p.setAa(aa, va, pp);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAa\" failed" << std::endl;

	p.setAa(r, aa, va, pp);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAa relative\" failed" << std::endl;

	p.setVa(va, pp);
	p.setAa(aa);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAa\" failed" << std::endl;

	p.setVa(r, va, pp);
	p.setAa(r, aa);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAa relative\" failed" << std::endl;

	p.setAs(as, vs, pm);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAs\" failed" << std::endl;

	p.setAs(r, as, vs, pm);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAs relative\" failed" << std::endl;

	p.setVs(vs, pm);
	p.setAs(as, vs, pm);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAs\" failed" << std::endl;

	p.setVs(r, vs, pm);
	p.setAs(r, as);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAs relative\" failed" << std::endl;


	r.setPm(relative_pm);
	r.setVs(relative_vs);
	r.setAs(relative_as);

	p.setPm(pm);
	p.setVs(vs);
	p.setAs(as);

	p.getPp(result);
	if (!(s_is_equal(3, result, pp, error)))std::cout << "\"coordinate:getPp\" failed" << std::endl;

	p.getRe(result);
	if (!(s_is_equal(3, result, re313, error)))std::cout << "\"coordinate:getRe 313\" failed" << std::endl;

	p.getRe(result, "321");
	if (!(s_is_equal(3, result, re321, error)))std::cout << "\"coordinate:getRe 321\" failed" << std::endl;

	p.getRq(result);
	if (!(s_is_equal(3, result, rq, error)))std::cout << "\"coordinate:getRq\" failed" << std::endl;

	p.getRm(result);
	if (!(s_is_equal(9, result, rm, error)))std::cout << "\"coordinate:getRm\" failed" << std::endl;

	p.getPe(result);
	if (!(s_is_equal(6, result, pe313, error)))std::cout << "\"coordinate:getPe\" failed" << std::endl;

	p.getPe(result, "321");
	if (!(s_is_equal(6, result, pe321, error)))std::cout << "\"coordinate:getPe 321\" failed" << std::endl;

	p.getPq(result);
	if (!(s_is_equal(7, result, pq, error)))std::cout << "\"coordinate:getPq\" failed" << std::endl;

	p.getPm(result);
	if (!(s_is_equal(16, result, pm, error)))std::cout << "\"coordinate:getPm\" failed" << std::endl;

	p.getVp(result, result2);
	if (!(s_is_equal(3, result, vp, error) && s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVp\" failed" << std::endl;

	p.getWe(result, result2);
	if (!(s_is_equal(3, result, we313, error) && s_is_equal(3, result2, re313, error)))std::cout << "\"coordinate:getWe 313\" failed" << std::endl;

	p.getWe(result, result2, "321");
	if (!(s_is_equal(3, result, we321, error) && s_is_equal(3, result2, re321, error)))std::cout << "\"coordinate:getWe 321\" failed" << std::endl;

	p.getWq(result, result2);
	if (!(s_is_equal(4, result, wq, error) && s_is_equal(4, result2, rq, error)))std::cout << "\"coordinate:getWq\" failed" << std::endl;

	p.getWm(result, result2);
	if (!(s_is_equal(9, result, wm, error) && s_is_equal(9, result2, rm, error)))std::cout << "\"coordinate:getWm\" failed" << std::endl;

	p.getWa(result, result3);
	if (!(s_is_equal(3, result, wa, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getWa\" failed" << std::endl;

	p.getVe(result, result2);
	if (!(s_is_equal(6, result, ve313, error) && s_is_equal(6, result2, pe313, error)))std::cout << "\"coordinate:getVe 313\" failed" << std::endl;

	p.getVe(result, result2, "321");
	if (!(s_is_equal(6, result, ve321, error) && s_is_equal(6, result2, pe321, error)))std::cout << "\"coordinate:getVe 321\" failed" << std::endl;

	p.getVq(result, result2);
	if (!(s_is_equal(7, result, vq, error) && s_is_equal(7, result2, pq, error)))std::cout << "\"coordinate:getVq\" failed" << std::endl;

	p.getVm(result, result2);
	if (!(s_is_equal(16, result, vm, error) && s_is_equal(16, result2, pm, error)))std::cout << "\"coordinate:getVm\" failed" << std::endl;

	p.getVa(result, result2);
	if (!(s_is_equal(6, result, va, error) && s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVa\" failed" << std::endl;

	p.getVs(result, result2);
	if (!(s_is_equal(6, result, vs, error) && s_is_equal(16, result2, pm, error)))std::cout << "\"coordinate:getVs\" failed" << std::endl;

	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, result, ap, error) && s_is_equal(3, result2, vp, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"coordinate:getAp\" failed" << std::endl;

	p.getXe(result, result2, result3);
	if (!(s_is_equal(3, result, xe313, error) && s_is_equal(3, result2, we313, error) && s_is_equal(3, result3, re313, error)))std::cout << "\"coordinate:getXe 313\" failed" << std::endl;

	p.getXe(result, result2, result3, "321");
	if (!(s_is_equal(3, result, xe321, error) && s_is_equal(3, result2, we321, error) && s_is_equal(3, result3, re321, error)))std::cout << "\"coordinate:getXe 313\" failed" << std::endl;

	p.getXq(result, result2, result3);
	if (!(s_is_equal(4, result, xq, error) && s_is_equal(4, result2, wq, error) && s_is_equal(4, result3, rq, error)))std::cout << "\"coordinate:getXq\" failed" << std::endl;

	p.getXm(result, result2, result3);
	if (!(s_is_equal(9, result, xm, error) && s_is_equal(9, result2, wm, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getXm\" failed" << std::endl;

	p.getXa(result, result2, result3);
	if (!(s_is_equal(3, result, xa, error) && s_is_equal(3, result2, wa, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getXm\" failed" << std::endl;

	p.getAe(result, result2, result3);
	if (!(s_is_equal(6, result, ae313, error) && s_is_equal(6, result2, ve313, error) && s_is_equal(6, result3, pe313, error)))std::cout << "\"coordinate:getAe 313\" failed" << std::endl;

	p.getAe(result, result2, result3, "321");
	if (!(s_is_equal(6, result, ae321, error) && s_is_equal(6, result2, ve321, error) && s_is_equal(6, result3, pe321, error)))std::cout << "\"coordinate:getAe 321\" failed" << std::endl;

	p.getAq(result, result2, result3);
	if (!(s_is_equal(7, result, aq, error) && s_is_equal(7, result2, vq, error) && s_is_equal(7, result3, pq, error)))std::cout << "\"coordinate:getAq\" failed" << std::endl;

	p.getAm(result, result2, result3);
	if (!(s_is_equal(16, result, am, error) && s_is_equal(16, result2, vm, error) && s_is_equal(16, result3, pm, error)))std::cout << "\"coordinate:getAm\" failed" << std::endl;

	p.getAa(result, result2, result3);
	if (!(s_is_equal(6, result, aa, error) && s_is_equal(3, result2, va, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"coordinate:getAa\" failed" << std::endl;

	p.getAs(result, result2, result3);
	if (!(s_is_equal(6, result, as, error) && s_is_equal(6, result2, vs, error) && s_is_equal(16, result3, pm, error)))std::cout << "\"coordinate:getAs\" failed" << std::endl;

	p.setPm(r, pm);
	p.setVs(r, vs);
	p.setAs(r, as);

	p.getPp(r, result);
	if (!(s_is_equal(3, result, pp, error)))std::cout << "\"coordinate:getPp\" failed" << std::endl;

	p.getRe(r, result);
	if (!(s_is_equal(3, result, re313, error)))std::cout << "\"coordinate:getRe\" failed" << std::endl;

	p.getRe(r, result, "321");
	if (!(s_is_equal(3, result, re321, error)))std::cout << "\"coordinate:getRe 321\" failed" << std::endl;

	p.getRq(r, result);
	if (!(s_is_equal(4, result, rq, error)))std::cout << "\"coordinate:getRq\" failed" << std::endl;

	p.getRm(r, result);
	if (!(s_is_equal(9, result, rm, error)))std::cout << "\"coordinate:getRm\" failed" << std::endl;

	p.getRm(r, result, 4);
	if (!(s_is_equal(3, 3, result, 4, rm, 3, error)))std::cout << "\"coordinate:getRm\" failed" << std::endl;

	p.getPe(r, result);
	if (!(s_is_equal(6, result, pe313, error)))std::cout << "\"coordinate:getPe\" failed" << std::endl;

	p.getPe(r, result, "321");
	if (!(s_is_equal(6, result, pe321, error)))std::cout << "\"coordinate:getPe 321\" failed" << std::endl;

	p.getPq(r, result);
	if (!(s_is_equal(7, result, pq, error)))std::cout << "\"coordinate:getPq\" failed" << std::endl;

	p.getPm(r, result);
	if (!(s_is_equal(16, result, pm, error)))std::cout << "\"coordinate:getPm\" failed" << std::endl;

	p.getVp(r, result, result2);
	if (!(s_is_equal(3, result, vp, error) && s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVp\" failed" << std::endl;

	p.getWe(r, result, result2);
	if (!(s_is_equal(3, result, we313, error) && s_is_equal(3, result2, re313, error)))std::cout << "\"coordinate:getWe 313\" failed" << std::endl;

	p.getWe(r, result, result2, "321");
	if (!(s_is_equal(3, result, we321, error) && s_is_equal(3, result2, re321, error)))std::cout << "\"coordinate:getWe 321\" failed" << std::endl;

	p.getWq(r, result, result2);
	if (!(s_is_equal(4, result, wq, error) && s_is_equal(4, result2, rq, error)))std::cout << "\"coordinate:getWq\" failed" << std::endl;

	p.getWm(r, result, result2);
	if (!(s_is_equal(9, result, wm, error) && s_is_equal(9, result2, rm, error)))std::cout << "\"coordinate:getWm\" failed" << std::endl;

	p.getWm(r, result, result2, 4, 4);
	if (!(s_is_equal(3, 3, result, 4, wm, 3, error) && s_is_equal(3, 3, result2, 4, rm, 3, error)))std::cout << "\"coordinate:getWm\" failed" << std::endl;

	p.getWa(r, result, result2);
	if (!(s_is_equal(3, result, wa, error) && s_is_equal(9, result2, rm, error)))std::cout << "\"coordinate:getWa\" failed" << std::endl;

	p.getWa(r, result, result2, 4);
	if (!(s_is_equal(3, result, wa, error) && s_is_equal(3, 3, result2, 4, rm, 3, error)))std::cout << "\"coordinate:getWa\" failed" << std::endl;

	p.getVe(r, result, result2);
	if (!(s_is_equal(6, result, ve313, error) && s_is_equal(6, result2, pe313, error)))std::cout << "\"coordinate:getVe 313\" failed" << std::endl;

	p.getVe(r, result, result2, "321");
	if (!(s_is_equal(6, result, ve321, error) && s_is_equal(6, result2, pe321, error)))std::cout << "\"coordinate:getVe 321\" failed" << std::endl;

	p.getVq(r, result, result2);
	if (!(s_is_equal(7, result, vq, error) && s_is_equal(7, result2, pq, error)))std::cout << "\"coordinate:getVq\" failed" << std::endl;

	p.getVm(r, result, result2);
	if (!(s_is_equal(16, result, vm, error) && s_is_equal(16, result2, pm, error)))std::cout << "\"coordinate:getVm\" failed" << std::endl;

	p.getVa(r, result, result2);
	if (!(s_is_equal(6, result, va, error) && s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVa\" failed" << std::endl;

	p.getVs(r, result, result2);
	if (!(s_is_equal(6, result, vs, error) && s_is_equal(16, result2, pm, error)))std::cout << "\"coordinate:getVs\" failed" << std::endl;

	p.getAp(r, result, result2, result3);
	if (!(s_is_equal(3, result, ap, error) && s_is_equal(3, result2, vp, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"coordinate:getAp\" failed" << std::endl;

	p.getXe(r, result, result2, result3);
	if (!(s_is_equal(3, result, xe313, error) && s_is_equal(3, result2, we313, error) && s_is_equal(3, result3, re313, error)))std::cout << "\"coordinate:getXe 313\" failed" << std::endl;

	p.getXe(r, result, result2, result3, "321");
	if (!(s_is_equal(3, result, xe321, error) && s_is_equal(3, result2, we321, error) && s_is_equal(3, result3, re321, error)))std::cout << "\"coordinate:getXe 321\" failed" << std::endl;

	p.getXq(r, result, result2, result3);
	if (!(s_is_equal(4, result, xq, error) && s_is_equal(4, result2, wq, error) && s_is_equal(4, result3, rq, error)))std::cout << "\"coordinate:getXq\" failed" << std::endl;

	p.getXm(r, result, result2, result3);
	if (!(s_is_equal(9, result, xm, error) && s_is_equal(9, result2, wm, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getXm\" failed" << std::endl;

	p.getXa(r, result, result2, result3);
	if (!(s_is_equal(3, result, xa, error) && s_is_equal(3, result2, wa, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getXa\" failed" << std::endl;

	p.getAe(r, result, result2, result3);
	if (!(s_is_equal(6, result, ae313, error) && s_is_equal(6, result2, ve313, error) && s_is_equal(6, result3, pe313, error)))std::cout << "\"coordinate:getAe 313\" failed" << std::endl;

	p.getAe(r, result, result2, result3, "321");
	if (!(s_is_equal(6, result, ae321, error) && s_is_equal(6, result2, ve321, error) && s_is_equal(6, result3, pe321, error)))std::cout << "\"coordinate:getAe 321\" failed" << std::endl;

	p.getAq(r, result, result2, result3);
	if (!(s_is_equal(7, result, aq, error) && s_is_equal(7, result2, vq, error) && s_is_equal(7, result3, pq, error)))std::cout << "\"coordinate:getAq\" failed" << std::endl;

	p.getAm(r, result, result2, result3);
	if (!(s_is_equal(16, result, am, error) && s_is_equal(16, result2, vm, error) && s_is_equal(16, result3, pm, error)))std::cout << "\"coordinate:getAm\" failed" << std::endl;

	p.getAa(r, result, result2, result3);
	if (!(s_is_equal(6, result, aa, error) && s_is_equal(6, result2, va, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"coordinate:getAa\" failed" << std::endl;

	p.getAs(r, result, result2, result3);
	if (!(s_is_equal(6, result, as, error) && s_is_equal(6, result2, vs, error) && s_is_equal(16, result3, pm, error)))std::cout << "\"coordinate:getAs\" failed" << std::endl;




	const double glb_im[36]{ 12.3, 0, 0,   0,   4.86680056754638, 1.19059364505643,
		0,   12.3, 0, -4.86680056754639, 0, -1.05294290764011,
		0, 0,   12.3, -1.19059364505643,1.05294290764011, 0,
		0, -4.86680056754639, -1.19059364505643, 6.37494518313694, -0.759574604730052, -1.02350564428774,
		4.86680056754638, 0,   1.05294290764011, -0.759574604730052,   7.88731016704578,   1.28815826950323,
		1.19059364505643, -1.05294290764011, 0, -1.02350564428774, 1.28815826950323,   6.06954528699909 };
	const double glb_fg[6]{ 0, -120.54,0,47.6946455619546,0,10.3188404948731 };
	const double glb_fv[6]{ 5.11681554065846,10.7408556783149,11.3489822760672, -3.55177495791515,2.99639902169271,-1.75750746281364 };
	const double prt_fg[6]{ 119.279123323887,-5.207328,16.5911444507421,0.45124518305897,3.31741898034387,-2.20293670541155 };
	const double prt_fv[6]{ -12.121294608774,-9.20587956029489,6.21877634351408,0.260711372733768,1.34261769787459,1.5411918276253 };
	const double prt_vs[6]{ -0.342374318815878, - 0.326839225000394, - 1.02664731827659, - 0.981437055212506,0.670440145936023, - 0.920485982149056 };
	const double prt_as[6]{ -1.91765344470424, - 2.61057756494702,0.488627252217732,0.786294961274511, - 2.28837157363616,1.03841805459299 };




	p.cptGlbIm(result, 7);
	if (!(s_is_equal(6, 6, result, 7, glb_im, 6, error)))std::cout << "\"part:cptGlbIm\" failed" << std::endl;

	p.cptGlbFg(result);
	if (!(s_is_equal(6, result, glb_fg, error)))std::cout << "\"part:cptGlbFg\" failed" << std::endl;

	p.cptGlbFv(result);
	if (!(s_is_equal(6, result, glb_fv, error)))std::cout << "\"part:cptGlbFv\" failed" << std::endl;

	p.cptPrtFg(result);
	if (!(s_is_equal(6, result, prt_fg, error)))std::cout << "\"part:cptPrtFg\" failed" << std::endl;

	p.cptPrtFv(result);
	if (!(s_is_equal(6, result, prt_fv, error)))std::cout << "\"part:cptPrtFv\" failed" << std::endl;

	p.cptPrtVs(result);
	if (!(s_is_equal(6, result, prt_vs, error)))std::cout << "\"part:cptPrtVs\" failed" << std::endl;

	p.cptPrtAs(result);
	if (!(s_is_equal(6, result, prt_as, error)))std::cout << "\"part:cptPrtAs\" failed" << std::endl;
}
void test_constraint()
{
	const double error = 1e-10;
	
	const double prt_pm_i[16]{ -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18,0.3,
		0,0,0,1 };
	const double prt_pm_j[16]{ -0.1224,   0.253539765421328,   0.959549804517774, -0.116974902258887,
		-0.989539765421329,   0.0432, -0.137640156385781, -0.0855499782797527,
		-0.0763498045177736, -0.966359843614219,   0.2456,   0.406691619606131,
		0,   0,   0,   1 };
	const double glb_pe_n[16]{ 0.58,0.64,0.35,1.25,0.31,0.22 };
	const double glb_vs_n[6]{ 0.15,0.3,0.289,0.12,4.35,2.31 };
	
	aris::dynamic::Model model;
	auto &prt_m = model.partPool().add<Part>("prt_m");
	auto &prt_n = model.partPool().add<Part>("prt_n");
	auto &mak_i = prt_m.markerPool().add<Marker>("mak_i", prt_pm_i);
	auto &mak_j = prt_n.markerPool().add<Marker>("mak_j", prt_pm_j);

	prt_n.setPe(glb_pe_n);
	prt_n.setVs(glb_vs_n);

	// test revolute joints //
	{
		const double relative_pe[6]{0,0,0,0.31,0,0};
		const double relative_vs[6]{ 0,0,0,0,0,0.515 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "321");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);

		auto &jnt = model.add<RevoluteJoint>("r1", mak_i, mak_j);

		const double glb_cmI[30]{ 0.772732834750084, -0.55499915414649,0.307993352194134,0,0,
			-0.0834135974649058,   0.392232918710641,   0.916076148165476, 0, 0,
			-0.629226618840194, -0.733572952320633,   0.256796779120235,   0,   0,
			-0.241746251737219, -0.626453518771242, -0.522335646172068,   0.772732834750084, -0.55499915414649,
			1.02612412552421,   0.171279303870418,   0.0200980273534624, -0.0834135974649058,   0.392232918710641,
			-0.432909166577701,   0.565536873993989,   0.554775584177545, -0.629226618840194, -0.733572952320633 };
		const double glb_cmJ[30]{ -0.772732834750084, 0.55499915414649,-0.307993352194134,0,0,
			0.0834135974649058,   -0.392232918710641,   -0.916076148165476, 0, 0,
			0.629226618840194, 0.733572952320633,   -0.256796779120235,   0,   0,
			0.241746251737219, 0.626453518771242, 0.522335646172068,   -0.772732834750084, 0.55499915414649,
			-1.02612412552421,   -0.171279303870418,   -0.0200980273534624, 0.0834135974649058,   -0.392232918710641,
			0.432909166577701,   -0.565536873993989,   -0.554775584177545, 0.629226618840194, 0.733572952320633 };
		const double prt_cmI[30]{ -0.22, - 0.975499782797526,0.000416847668728071,0,0,
			0.175499782797526, - 0.04, - 0.983666521865018,   0,   0,
			0.959583152331272, - 0.216333478134982,   0.18,   0,   0,
			0.139266695626997, - 0.0312666956269964,   0.331099956559505, - 0.22, - 0.975499782797526,
			- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816,0.175499782797526, - 0.04,
			0.0615499782797526,   0.191099956559505, - 0.0984500217202474,0.959583152331272, - 0.216333478134982, };
		const double prt_cmJ[30]{ 0.0392211338303902, - 0.278793607012357, - 0.959549804517775, - 0, - 0,
			0.929193404253209, - 0.343008461765058,   0.137640156385781,   0,   0,
			0.367506898103141,   0.897005752404413, - 0.2456, - 0, - 0,
			- 0.409335377653455,   0.0627598442188279, - 0.0349660234578675,   0.0392211338303902, - 0.278793607012357,
			0.0589399899253518, - 0.008455863358525, - 0.418969900086863,   0.929193404253209, - 0.343008461765058,
			- 0.105336940494824,   0.0162725942644976, - 0.0981899087749608,   0.367506898103141,   0.897005752404413 };
		const double ce[6]{ 0,0,0,0,0,0 };
		const double ca[6]{ 0.926329546106927,-1.72197953069011,0,	-0.0282942143129791,	0.887671869636436 };

		double result1[42], result2[48];

		jnt.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 5, glb_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, glb_cmJ, jnt.dim(), error))std::cout << "\"RevoluteJoint:cptGlbCm\" failed" << std::endl;

		jnt.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 6, prt_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, prt_cmJ, jnt.dim(), error))std::cout << "\"RevoluteJoint:cptPrtCm\" failed" << std::endl;

		jnt.cptCa(result1);
		if (!s_is_equal(jnt.dim(), result1, ca, error))std::cout << "\"RevoluteJoint:cptCa\" failed" << std::endl;

		jnt.cptCp(result1);
		if (!s_is_equal(jnt.dim(), result1, ce, error))std::cout << "\"RevoluteJoint:cptCp\" failed" << std::endl;
	}

	// test prismatic joints //
	{
		const double relative_pe[6]{ 0,0,0.326,0,0,0 };
		const double relative_vs[6]{ 0,0,0.518,0,0,0 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "321");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);

		auto &jnt = model.add<PrismaticJoint>("p1", mak_i, mak_j);

		const double glb_cmI[30]{ 0.905206704276648, - 0.292815500847941,0,0,0,
			- 0.199091608400864,0.348090537398911, - 0, - 0, - 0,
			- 0.375450867620476, - 0.890557162812419,   0,   0,   0,
			- 0.134575867968274, - 0.96543688341334,   0.905206704276648, - 0.292815500847941,   0.307993352194134,
			1.03843973590868,   0.541046921795271, - 0.199091608400864,   0.348090537398911,   0.916076148165476,
			- 0.875117474759134,   0.528914052896916, - 0.375450867620476, - 0.890557162812419,   0.256796779120235};
		const double glb_cmJ[30]{ -0.905206704276648, 0.292815500847941,0,0,0,
			0.199091608400864,-0.348090537398911, -0, -0, -0,
			0.375450867620476, 0.890557162812419,   0,   0,   0,
			0.134575867968274, 0.96543688341334,   -0.905206704276648, 0.292815500847941,   -0.307993352194134,
			-1.03843973590868,   -0.541046921795271, 0.199091608400864,   -0.348090537398911,   -0.916076148165476,
			0.875117474759134,   -0.528914052896916, 0.375450867620476, 0.890557162812419,   -0.256796779120235, };
		const double prt_cmI[30]{ -0.22, - 0.975499782797526 ,  0,   0,   0,
			0.175499782797526, - 0.04,   0,   0,   0,
			0.959583152331272, - 0.216333478134982,   0,   0,   0,
			0.139266695626997, - 0.0312666956269964, - 0.22, - 0.975499782797526,   0.000416847668728071,
			- 0.161958315233127, - 0.27101658702576,   0.175499782797526, - 0.04, - 0.983666521865018,
			0.0615499782797526,   0.191099956559505,   0.959583152331272, - 0.216333478134982,   0.18 };
		const double prt_cmJ[30]{ 0.1224, - 0.253539765421328, - 0, - 0, - 0,
			0.989539765421329, - 0.0431999999999995, - 0, - 0, - 0,
			0.0763498045177739,   0.966359843614218, - 0, - 0, - 0,
			- 0.491623217509383, - 0.105005385664637,   0.1224, - 0.253539765421328, - 0.959549804517775,
			0.0446268651607419, - 0.312662613107425,   0.989539765421329, - 0.0431999999999995,   0.137640156385781,
			0.209753309018236, - 0.0415270419200584,   0.0763498045177739,   0.966359843614218, - 0.2456 };
		const double ce[6]{ 0,0,0,0,0,0 };
		const double ca[6]{ -0.299471893489825,   0.841602471649138, - 0, - 0, - 0 };


		double result1[42], result2[48];


		jnt.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 5, glb_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, glb_cmJ, jnt.dim(), error))std::cout << "\"PrismaticJoint:cptGlbCm\" failed" << std::endl;

		jnt.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 6, prt_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, prt_cmJ, jnt.dim(), error))std::cout << "\"PrismaticJoint:cptPrtCm\" failed" << std::endl;

		jnt.cptCa(result1);
		if (!s_is_equal(jnt.dim(), result1, ca, error))std::cout << "\"PrismaticJoint:cptCa\" failed" << std::endl;

		jnt.cptCp(result1);
		if (!s_is_equal(jnt.dim(), result1, ce, error))std::cout << "\"PrismaticJoint:cptCp\" failed" << std::endl;
	}

	// test spherical joints //
	{
		const double relative_pe[6]{ 0,0,0,0.221,0.654,0.123 };
		const double relative_vs[6]{ 0,0,0,0.514,0.356,0.777 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "321");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);

		auto &jnt = model.add<SphericalJoint>("s1", mak_i, mak_j);

		const double glb_cmI[]{ 0.462635502589964, - 0.389341212341349,   0.796480892498936,
			- 0.650918208225164,   0.460769198258918,   0.603321831311263,
			- 0.601891915500013, - 0.797562014083169, - 0.0402610947109546,
			0.170858464032355, - 0.705263081708994, - 0.443994548974076,
			0.786857634821923,   0.336935956217407,   0.591608380223745,
			- 0.719622176761417,   0.538939525796233,   0.0818923769414457 };
		const double glb_cmJ[]{ -0.462635502589964, 0.389341212341349,   -0.796480892498936,
			0.650918208225164,   -0.460769198258918,   -0.603321831311263,
			0.601891915500013, 0.797562014083169, 0.0402610947109546,
			-0.170858464032355, 0.705263081708994, 0.443994548974076,
			-0.786857634821923,   -0.336935956217407,   -0.591608380223745,
			0.719622176761417,   -0.538939525796233,   -0.0818923769414457 };
		const double prt_cmI[]{ -0.22, - 0.975499782797526,   0.000416847668728071,
			0.175499782797526, - 0.04, - 0.983666521865018,
			0.959583152331272, - 0.216333478134982,   0.18,
			0.139266695626997, - 0.0312666956269964,   0.331099956559505,
			- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816,
			0.0615499782797526,   0.191099956559505, - 0.0984500217202474};
		const double prt_cmJ[]{ 0.634429008738031, - 0.360802037735136, - 0.683609334662608,
			0.675002783167594, - 0.172345515887459,   0.717403837367413,
			0.376657769849079,   0.916580008902473, - 0.134201384838831,
			- 0.306741039154564, - 0.00832192286513729, - 0.280281202972532,
			0.302076466901929, - 0.039518308129878, - 0.293716381366335,
			- 0.0246829966679223, - 0.0107065066158384, - 0.142401007488425 };
		const double ce[6]{ 0,0,0,0,0,0 };
		const double ca[]{ 1.25497709355873, - 1.42034889864347,   0.552653481810815 };

		double result1[42], result2[48];

		jnt.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 5, glb_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, glb_cmJ, jnt.dim(), error))std::cout << "\"SphericalJoint:cptGlbCm\" failed" << std::endl;

		jnt.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 6, prt_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, prt_cmJ, jnt.dim(), error))std::cout << "\"SphericalJoint:cptPrtCm\" failed" << std::endl;

		jnt.cptCa(result1);
		if (!s_is_equal(jnt.dim(), result1, ca, error))std::cout << "\"SphericalJoint:cptCa\" failed" << std::endl;

		jnt.cptCp(result1);
		if (!s_is_equal(jnt.dim(), result1, ce, error))std::cout << "\"SphericalJoint:cptCp\" failed" << std::endl;
	}
	
	// test universal joints //
	{
		const double relative_pe[6]{ 0,0,0,0.856,PI / 2,0.972 };
		const double relative_ve[6]{ 0,0,0,0.157,0,0.895 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "313");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVe(mak_j, relative_ve, nullptr, "313");

		auto &jnt = model.add<UniversalJoint>("u1", mak_i, mak_j);

		const double glb_cmI[]{ 0.464190255001298, - 0.133832675696459,   0.875566229406867,   0,
			0.831313762500364,   0.40698899884696, - 0.378519990350626,   0,
			- 0.305687480017286,   0.903575547330423,   0.300177272336208,   0,
			- 0.731260909855271,   0.144942403197629,   0.409840176342917, - 0.372187953763235,
			0.561015416631454, - 0.786498730845814,   0.386461770813631, - 0.132389983258812,
			0.415246475240747,   0.375723271514129, - 0.708110612260744,   0.918666979599392 };
		const double glb_cmJ[]{ -0.464190255001298,   0.133832675696459, - 0.875566229406867, - 0,
			- 0.831313762500364, - 0.40698899884696,   0.378519990350626, - 0,
			0.305687480017286, - 0.903575547330423, - 0.300177272336208, - 0,
			0.731260909855271, - 0.144942403197629, - 0.409840176342917,   0.372187953763235,
			- 0.561015416631454,   0.786498730845814, - 0.386461770813631,   0.132389983258812,
			- 0.415246475240747, - 0.375723271514129,   0.708110612260744, - 0.918666979599392 };
		const double prt_cmI[]{ -0.22, - 0.975499782797526,   0.000416847668728071,   0,
			0.175499782797526, - 0.04, - 0.983666521865018,   0,
			0.959583152331272, - 0.216333478134982,   0.18,   0,
			0.139266695626997, - 0.0312666956269964,   0.331099956559505, - 0.681774424596104,
			- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816, - 0.131960798655384,
			0.0615499782797526,   0.191099956559505, - 0.0984500217202474, - 0.719562354202113 };
		const double prt_cmJ[]{ -0.855308832662975, - 0.448953753689671,   0.258625845221729, - 0,
			0.460889922612995, - 0.431229074578346,   0.775642936196864, - 0,
			0.23670082383145, - 0.782612300111783, - 0.575752297182787,   0,
			- 0.207689819425203,   0.242329716037551, - 0.266191885439452,   0.111251268906238,
			- 0.320158878686852, - 0.274131726528483,   0.0378323951769434, - 0.615981511798859,
			- 0.127084205706559,   0.0120349949732274, - 0.0686053212081227, - 0.779865329585016 };
		const double ce[6]{ 0,0,0,0,0,0 };
		const double ca[]{ 0.297342839758157,3.33820400487961,3.34479764841342,5.30996702214242 };


		double result1[42], result2[48];


		jnt.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 5, glb_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, glb_cmJ, jnt.dim(), error))std::cout << "\"UniversalJoint:cptGlbCm\" failed" << std::endl;

		jnt.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 6, prt_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, prt_cmJ, jnt.dim(), error))std::cout << "\"UniversalJoint:cptPrtCm\" failed" << std::endl;

		jnt.cptCa(result1);
		if (!s_is_equal(jnt.dim(), result1, ca, error))std::cout << "\"UniversalJoint:cptCa\" failed" << std::endl;

		jnt.cptCp(result1);
		if (!s_is_equal(jnt.dim(), result1, ce, error))std::cout << "\"UniversalJoint:cptCp\" failed" << std::endl;
	}
	
	// test motion //
	{
		const double relative_pe[6]{ 0,0,0.521,0,0,0 };
		const double relative_vs[6]{ 0,0,0.689,0,0,0 };
		const double relative_as[6]{ 0,0,0.123,0,0,0 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "123");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);
		prt_m.setAs(mak_j, relative_as);

		auto &mot = model.add<Motion>("m1", mak_i, mak_j);

		const double glb_cmI[]{ 0.307993352194134,
			0.916076148165476,
			0.256796779120235,
			-0.522335646172068,
			0.0200980273534624,
			0.554775584177545 };
		const double glb_cmJ[]{ -0.307993352194134,
			- 0.916076148165476,
			- 0.256796779120235,
			0.522335646172068,
			- 0.0200980273534624,
			- 0.554775584177545 };
		const double prt_cmI[]{ 0.000416847668728071,
			- 0.983666521865018,
			0.18,
			0.331099956559505,
			- 0.0178749456993816,
			- 0.0984500217202474 };
		const double prt_cmJ[]{ -0.959549804517775,
			0.137640156385781,
			- 0.2456,
			- 0.0349660234578675,
			- 0.418969900086863,
			- 0.0981899087749607 };
		const double ce[6]{ -0.521,0,0,0,0,0 };
		const double ca[]{ 0 };


		double result1[42], result2[48];


		mot.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, mot.dim(), result1, 5, glb_cmI, mot.dim(), error) || !s_is_equal(6, mot.dim(), result2, 7, glb_cmJ, mot.dim(), error))std::cout << "\"Motion:cptGlbCm\" failed" << std::endl;

		mot.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, mot.dim(), result1, 6, prt_cmI, mot.dim(), error) || !s_is_equal(6, mot.dim(), result2, 7, prt_cmJ, mot.dim(), error))std::cout << "\"Motion:cptPrtCm\" failed" << std::endl;

		mot.cptCa(result1);
		if (!s_is_equal(mot.dim(), result1, ca, error))std::cout << "\"Motion:cptCa\" failed" << std::endl;

		mot.cptCp(result1);
		if (!s_is_equal(mot.dim(), result1, ce, error))std::cout << "\"Motion:cptCp\" failed" << std::endl;

		mot.updMp();
		if (!s_is_equal(mot.mp(), 0.521, error))std::cout << "\"Motion:updMp\" failed" << std::endl;

		mot.updMv();
		if (!s_is_equal(mot.mv(), 0.689, error))std::cout << "\"Motion:updMv\" failed" << std::endl;

		mot.updMa();
		if (!s_is_equal(mot.ma(), 0.123, error))std::cout << "\"Motion:updMa\" failed" << std::endl;
	}

	// test general motion //
	{
		const double relative_pe[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
		const double relative_vs[6]{ 0,0,0.689,0,0,0 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "123");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);

		auto &mot = model.add<GeneralMotion>("m1", mak_i, mak_j);
		
		const double mpm_default[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		const double mpe313[6] = { 0.1, 0.2, 0.3,0.000423769269879415,   1.38980987554835,   1.79253453841257 };
		const double mpe321[6] = { 0.1, 0.2, 0.3,2.46823966120654, -1.28551725555848,  5.40636866254317 };
		const double mpq[7] = { 0.1, 0.2, 0.3,0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
		const double mpm[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
			0.175499782797526, -0.04, -0.983666521865018, 0.2,
			0.959583152331272, -0.216333478134982,   0.18,0.3,
			0,0,0,1 };

		const double mvs_default[6]{ 0,0,0,0,0,0 };
		const double mve313[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-0.644213536852877, -0.245050866834802, -1.27836042009784 };
		const double mve321[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-4.19969388864156, -0.83045134600268,   3.46543753721832 };
		const double mvq[7] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, 0.1, 0.2, -0.4, -(mpq[3] * 0.1 + mpq[4] * 0.2 - mpq[5] * 0.4) / mpq[6] };
		const double mvm[16] = { 1.36, -0.30698536874045, -0.633709981238717,0.307558670154491,
			0.426985368740452,   0.8,   0.0436487757967661,1.2433000508379,
			0.233709981238715,   1.23635122420323,   0.24 , -1.04895965543501,
			0,0,0,0 };
		const double mva[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
		const double mvs[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
		
		const double mas_default[6]{ 0,0,0,0,0,0 };
		const double mae313[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 1.51734920338156,   1.71538128045296,   1.3693196878275 };
		const double mae321[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -15.6049676192293,   4.50445705187534,   16.9352080725126 };
		const double maq[7] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -0.033,   0.022, 0.011,   -(mvq[3] * mvq[3] + mvq[4] * mvq[4] + mvq[5] * mvq[5] + mvq[6] * mvq[6] + mpq[3] * (-0.033) + mpq[4] * (0.022) + mpq[5] * (0.011)) / mpq[6] };
		const double mam[16] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,2.2628985000154,
			-2.32024759895695, -0.653600000000002,   0.450521351741563,-0.843606386309081,
			-1.92944395313478, -1.05972135174157, -0.103200000000001,-0.248846478459814,
			0,0,0,0 };
		const double maa[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 0.904633672502324, -1.24440604199266,   1.45568007018557 };
		const double mas[6] = { 3.15925342342501, -0.192390604845803,   0.136512424183815,   0.904633672502324, -1.24440604199266,   1.45568007018557 };

		double result[36];

		mot.setMpm(mpm_default);
		if (!s_is_equal(16, *mot.mpm(), mpm_default, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.setMpe(mpe313, "313");
		if (!s_is_equal(16, *mot.mpm(), mpm, error))std::cout << "\"GeneralMotion:setMpe 313\" failed" << std::endl;

		mot.setMpm(mpm_default);
		if (!s_is_equal(16, *mot.mpm(), mpm_default, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.setMpe(mpe321, "321");
		if (!s_is_equal(16, *mot.mpm(), mpm, error))std::cout << "\"GeneralMotion:setMpe 321\" failed" << std::endl;

		mot.setMpm(mpm_default);
		if (!s_is_equal(16, *mot.mpm(), mpm_default, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.setMpq(mpq);
		if (!s_is_equal(16, *mot.mpm(), mpm, error))std::cout << "\"GeneralMotion:setMpq\" failed" << std::endl;
		
		mot.setMpm(mpm_default);
		if (!s_is_equal(16, *mot.mpm(), mpm_default, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.setMpm(mpm);
		if (!s_is_equal(16, *mot.mpm(), mpm, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.getMpe(result, "313");
		if (!s_is_equal(6, result, mpe313, error))std::cout << "\"GeneralMotion:getMpe\" failed" << std::endl;

		mot.getMpe(result, "321");
		if (!s_is_equal(6, result, mpe321, error))std::cout << "\"GeneralMotion:getMpe\" failed" << std::endl;

		mot.getMpq(result);
		if (!s_is_equal(7, result, mpq, error))std::cout << "\"GeneralMotion:getMpq\" failed" << std::endl;

		mot.getMpm(result);
		if (!s_is_equal(16, result, mpm, error))std::cout << "\"GeneralMotion:getMpm\" failed" << std::endl;


		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMve(mve313, "313");
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMve 313\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMve(mve321, "321");
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMve 321\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMvq(mvq);
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMvq\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMvm(mvm);
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMvm\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMva(mva);
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMva\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMvs(mvs);
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.getMve(result, "313");
		if (!s_is_equal(6, result, mve313, error))std::cout << "\"GeneralMotion:getMve\" failed" << std::endl;

		mot.getMve(result, "321");
		if (!s_is_equal(6, result, mve321, error))std::cout << "\"GeneralMotion:getMve\" failed" << std::endl;

		mot.getMvq(result);
		if (!s_is_equal(7, result, mvq, error))std::cout << "\"GeneralMotion:getMvq\" failed" << std::endl;

		mot.getMvm(result);
		if (!s_is_equal(16, result, mvm, error))std::cout << "\"GeneralMotion:getMvm\" failed" << std::endl;

		mot.getMva(result);
		if (!s_is_equal(6, result, mva, error))std::cout << "\"GeneralMotion:getMva\" failed" << std::endl;

		mot.getMvs(result);
		if (!s_is_equal(6, result, mvs, error))std::cout << "\"GeneralMotion:getMvs\" failed" << std::endl;


		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMae(mae313, "313");
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMae 313\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMae(mae321, "321");
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMae 321\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMaq(maq);
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMaq\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMam(mam);
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMam\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMaa(maa);
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMaa\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMas(mas);
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;
		
		mot.getMae(result, "313");
		if (!s_is_equal(6, result, mae313, error))std::cout << "\"GeneralMotion:getMae\" failed" << std::endl;

		mot.getMae(result, "321");
		if (!s_is_equal(6, result, mae321, error))std::cout << "\"GeneralMotion:getMae\" failed" << std::endl;

		mot.getMaq(result);
		if (!s_is_equal(7, result, maq, error))std::cout << "\"GeneralMotion:getMaq\" failed" << std::endl;

		mot.getMam(result);
		if (!s_is_equal(16, result, mam, error))std::cout << "\"GeneralMotion:getMam\" failed" << std::endl;

		mot.getMaa(result);
		if (!s_is_equal(6, result, maa, error))std::cout << "\"GeneralMotion:getMaa\" failed" << std::endl;

		mot.getMas(result);
		if (!s_is_equal(6, result, mas, error))std::cout << "\"GeneralMotion:getMas\" failed" << std::endl;

		const double glb_cmI[]{ 0.413710949602281, - 0.464491586871515,   0.783001159580726,   0,   0,   0,
			- 0.0419709404545899,   0.849409663646283,   0.526062414036267, 0, 0, 0,
			- 0.90944031708328, - 0.25050107590565,   0.331914929814216,   0,   0,   0,
			- 0.705694160625697, - 0.681200783491694, - 0.0312370311099321,   0.413710949602281, - 0.464491586871515,   0.783001159580726,
			1.04378672046201, - 0.0395661962367909,   0.147162423437692, - 0.0419709404545899,   0.849409663646283,   0.526062414036267,
			- 0.369196422575466,   1.12895372781328, - 0.159552895610227, - 0.90944031708328, - 0.25050107590565,   0.331914929814216 };
		const double glb_cmJ[]{ -0.413710949602281, 0.464491586871515,   -0.783001159580726,   0,   0,   0,
			0.0419709404545899,  - 0.849409663646283,   -0.526062414036267, 0, 0, 0,
			0.90944031708328, 0.25050107590565,   -0.331914929814216,   0,   0,   0,
			0.705694160625697, 0.681200783491694, 0.0312370311099321,   -0.413710949602281, 0.464491586871515,   -0.783001159580726,
			-1.04378672046201, 0.0395661962367909,   -0.147162423437692, 0.0419709404545899,   -0.849409663646283,   -0.526062414036267,
			0.369196422575466,   -1.12895372781328, 0.159552895610227, 0.90944031708328, 0.25050107590565,   -0.331914929814216 };
		const double prt_cmI[]{ -0.22, - 0.975499782797526,   0.000416847668728071,   0,   0,   0,
			0.175499782797526, - 0.04, - 0.983666521865018,   0,   0,   0,
			0.959583152331272, - 0.216333478134982,   0.18,   0,   0,   0,
			0.139266695626997, - 0.0312666956269964,   0.331099956559505, - 0.22, - 0.975499782797526,   0.000416847668728071,
			- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816,   0.175499782797526, - 0.04, - 0.983666521865018,
			0.0615499782797526,   0.191099956559505, - 0.0984500217202474,   0.959583152331272, - 0.216333478134982,   0.18 };
		const double prt_cmJ[]{ 0.056450322927895, - 0.774310621053255, - 0.630282812049845,   0,   0,   0,
			0.667701575653235, - 0.440066920670111,   0.600429605534333, - 0, - 0, - 0,
			0.742285637010123,   0.454735271840714, - 0.492166501940581, - 0, - 0, - 0,
			- 0.347790781866912,   0.0242346634708827, - 0.0609219520773668,   0.056450322927895, - 0.774310621053255, - 0.630282812049845,
			- 0.139627559696223, - 0.311594926010587, - 0.0731027876832712,   0.667701575653235, - 0.440066920670111,   0.600429605534333,
			0.152047187678477, - 0.260277725507541, - 0.0111649587681636,   0.742285637010123,   0.454735271840714, - 0.492166501940581 };
		const double ce[6]{ 0,0,0,0,0,0 };
		const double ca[]{ 0,0,0,0,0,0 };


		double result1[42], result2[48];

		mot.setMpe(relative_pe, "123");
		

		mot.cptGlbCm(result1, 6, result2, 7);
		if (!s_is_equal(6, mot.dim(), result1, 6, glb_cmI, mot.dim(), error) || !s_is_equal(6, mot.dim(), result2, 7, glb_cmJ, mot.dim(), error))std::cout << "\"GeneralMotion:cptGlbCm\" failed" << std::endl;

		mot.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, mot.dim(), result1, 6, prt_cmI, mot.dim(), error) || !s_is_equal(6, mot.dim(), result2, 7, prt_cmJ, mot.dim(), error))std::cout << "\"GeneralMotion:cptPrtCm\" failed" << std::endl;

		mot.cptCp(result1);
		if (!s_is_equal(mot.dim(), result1, ce, error))std::cout << "\"Motion:cptCp\" failed" << std::endl;

		//mot.cptCa(result1);
		//if (!s_is_equal(mot.dim(), result1, ca, error))std::cout << "\"Motion:cptCa\" failed" << std::endl;



		//mot.updMp();
		//if (std::abs(mot.mp() - 0.521)>error)std::cout << "\"Motion:updMp\" failed" << std::endl;

		//mot.updMv();
		//if (std::abs(mot.mv() - 0.689)>error)std::cout << "\"Motion:updMv\" failed" << std::endl;

		//mot.updGlbCm();
		//mot.updPrtCm();
		//mot.updCa();
		//mot.updCe();

		//dlmwrite("C:\\Users\\py033\\Desktop\\cm.txt", mot.ca(), 1, mot.dim());

	}
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
		m.loadXml(xml_doc);

		for (auto &mot : m.motionPool())mot.activate(true);
		m.generalMotionPool().at(0).activate(false);

		auto &gs = static_cast<GroundDividedSolver&>(*m.solverPool().findByName("gs"));
		auto &ps = static_cast<PartDividedSolver&>(*m.solverPool().findByName("ps"));
		auto &ds = static_cast<DiagSolver&>(*m.solverPool().findByName("ds"));
		auto &gcs = m.solverPool().add<GroundCombineSolver>("gcs");

		gs.allocateMemory();

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
void test_solver_3R()
{
	const double error = 1e-10;

	try
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_file_3R);
		Model m;
		m.loadXml(xml_doc);

		for (auto &mot : m.motionPool())mot.activate(true);
		m.generalMotionPool().at(0).activate(false);

		auto &gs = static_cast<GroundDividedSolver&>(*m.solverPool().findByName("gs"));
		auto &ps = static_cast<PartDividedSolver&>(*m.solverPool().findByName("ps"));
		auto &ds = static_cast<DiagSolver&>(*m.solverPool().findByName("ds"));
		auto &gcs = m.solverPool().add<GroundCombineSolver>("gcs");

		const double input_origin_p[3]{ 0.0, 0.0, 0.0 };
		const double input_origin_v[3]{ 0.0, 0.0, 0.0 };
		const double input_origin_a[3]{ 0.0, 0.0, 0.0 };
		const double input_origin_mf[3]{ -29.4,	-9.8,	0.0 };
		const double output_origin_pm[16]{ 1.0 , 0.0 , 0.0 , 3.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };

		const double input_p[3]{ 0.585, 0.685, -0.312 };
		const double input_v[3]{ 0.235, 0.235, 0.235 };
		const double input_a[3]{ -1.567, -1.567, -1.567};
		const double input_mf[3]{ 0.17105260350807,	9.24402272506392,	4.70099999999998 };
		const double input_p2[3]{ 1.27, -0.685, 0.373 };
		const double input_v2[3]{ 0.47, -0.235, 0.47 };
		const double input_a2[3]{ -3.134, 1.567, -3.134 };
		const double output_pm[16]{ 0.5751572210772128, - 0.8180428907109568,   0.0000000000000000,   1.7051501798895701,
			0.8180428907109568,   0.5751572210772128,   0.0000000000000000,   2.3253431286258799,
			0.0000000000000000,   0.0000000000000000,   1.0000000000000000,   0.0000000000000000,
			0.0000000000000000,   0.0000000000000000,   0.0000000000000000,   1.0000000000000000, };
		const double output_va[6]{ -1.1553844949236325 , 0.7406601913177889 , 0.0 , 0.0 , 0.0 , 0.705 };
		const double output_aa[6]{ 7.3068444301678799 ,-5.5868499482603751 , 0.0 , 0.0 , 0.0 ,-4.701 };
		const double output_mfs[6]{ 0.17105260350807,	9.24402272506392,	4.70099999999998 };

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
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
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
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
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
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
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
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
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
			s.allocateMemory();
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
			s.allocateMemory();
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
			s.allocateMemory();
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
			s.allocateMemory();
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
			s.allocateMemory();
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
			s.allocateMemory();
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

		m.simulatorPool().add<SolverSimulator>("simulator", ds);
		m.simResultPool().add<SimResult>("result1");
		m.simResultPool().front().allocateMemory();
		m.simResultPool().front().allocateMemory();
		m.simResultPool().front().record();
		m.simResultPool().front().record();
		m.simResultPool().front().record();

		m.saveXml("C:\\Users\\py033\\Desktop\\m.xml");


		Model m2;
		m2.loadXml("C:\\Users\\py033\\Desktop\\m.xml");
		m2.saveXml("C:\\Users\\py033\\Desktop\\m2.xml");
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
		const double input_origin_mf[6]{ -34.8372347443935, -0.179465241344625, -26.1146353297101,3.12638803734444e-13,5.6843418860808e-13, -9.9475983006414e-14 };
		const double output_origin_pm[16]{ 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ -1.53575836143848,-38.0002468240561,-16.8933419758238,-8.4241178308505,16.6881665634178,-51.5626922658449 };

		const double input_p[6]{ -0.084321840829742,0.111235847475406,0.163501201249858,0.41316722587035, -0.0861578092597486,0.229246197281016 };
		const double input_v[6]{ 0.93426722257942, -0.024823760537999, -0.89419018046124,   0.245922301638701, -1.23100367003297, -0.48185561218356 };
		const double input_a[6]{ 0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337,   0.131806583011732, -1.65802060177352 };
		const double input_mf[6]{ -24.6359418510515,3.06678992657553, -13.4565070365958,   15.0336821069307,   0.786112551012351,   1.93281931696021 };
		const double output_pm[16]{ 0.863013488544127, -0.284074444773496,   0.417743256579356, -0.137731283515364,
			0.387677110267304,   0.902605554641921, -0.187108714132569, -0.343275971674581,
			-0.323904579723239,   0.323426842664891,   0.889089928341408, -0.0474940394315194,
			0,   0,   0,   1 };
		const double output_va[6]{ -1.93242030056314,   0.500930573127293,   0.577926916892486, -0.399682310201935, -0.66053331463003, -0.857440373970742 };
		const double output_aa[6]{ 1.07075600145293,   0.349116022890415,   2.0925775293411, -1.77982973680254, -0.927893632540704,   0.0659817357654945 };
		const double output_mfs[6]{ -8.44990411304192, -54.7768126462764, -23.2058019399381, -18.6214939645874,   51.751313528282, -82.047228392192 };

		double result1[16], result2[16], result3[16], result4[16];

		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_file_6R);
		Model m;
		m.loadXml(xml_doc);

		auto test_forward = [&](Solver &s)->void
		{
			// set topology //
			for (auto &mot : m.motionPool())mot.activate(true);
			m.generalMotionPool().at(0).activate(false);
			// set input origin //
			for (aris::Size i = 0; i < 6; ++i)
			{
				m.motionPool().at(i).setMp(input_origin_p[i]);
				m.motionPool().at(i).setMv(input_origin_v[i]);
				m.motionPool().at(i).setMa(input_origin_a[i]);
			}
			// compute //
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
			s.dynAccAndFce();

			// get result //
			m.generalMotionPool().at(0).updMpm();
			m.generalMotionPool().at(0).getMpm(result1);
			m.generalMotionPool().at(0).updMvs();
			m.generalMotionPool().at(0).getMva(result2);
			m.generalMotionPool().at(0).updMas();
			m.generalMotionPool().at(0).getMaa(result3);
			for (int i = 0; i < 6; ++i)result4[i] = m.motionPool().at(i).mf();
			// check //
			if (!s_is_equal(16, result1, output_origin_pm, error))std::cout << s.type() << "::kinPos() forward origin failed" << std::endl;
			if (!s_is_equal(6, result2, output_origin_va, error))std::cout << s.type() << "::kinVel() forward origin failed" << std::endl;
			if (!s_is_equal(6, result3, output_origin_aa, error))std::cout << s.type() << "::kinAcc() forward origin failed" << std::endl;
			if (!s_is_equal(6, result4, input_origin_mf, 1e-9))std::cout << s.type() << "::dynFce() forward origin failed" << std::endl;

			// set input //
			for (aris::Size i = 0; i < 6; ++i)
			{
				m.motionPool().at(i).setMp(input_p[i]);
				m.motionPool().at(i).setMv(input_v[i]);
				m.motionPool().at(i).setMa(input_a[i]);
			}
			// compute //
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
			s.dynAccAndFce();
			// get result //
			m.generalMotionPool().at(0).updMpm();
			m.generalMotionPool().at(0).getMpm(result1);
			m.generalMotionPool().at(0).updMvs();
			m.generalMotionPool().at(0).getMva(result2);
			m.generalMotionPool().at(0).updMas();
			m.generalMotionPool().at(0).getMaa(result3);
			for (int i = 0; i < 6; ++i)result4[i] = m.motionPool().at(i).mf();
			// check //
			if (!s_is_equal(16, result1, output_pm, error))std::cout << s.type() << "::kinPos() forward failed" << std::endl;
			if (!s_is_equal(6, result2, output_va, error))std::cout << s.type() << "::kinVel() forward failed" << std::endl;
			if (!s_is_equal(6, result3, output_aa, error))std::cout << s.type() << "::kinAcc() forward failed" << std::endl;
			if (!s_is_equal(6, result4, input_mf, 1e-9))std::cout << s.type() << "::dynFce() forward failed" << std::endl;

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
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
			s.dynAccAndFce();

			// get result //
			for (aris::Size i = 0; i < 6; ++i)
			{
				m.motionPool().at(i).updMp();
				m.motionPool().at(i).updMv();
				m.motionPool().at(i).updMa();
				result1[i] = m.motionPool().at(i).mp();
				result2[i] = m.motionPool().at(i).mv();
				result3[i] = m.motionPool().at(i).ma();
			}
			// check //
			if (!s_is_equal(6, result1, input_origin_p, error))std::cout << s.type() << "::kinPos() inverse origin failed" << std::endl;
			if (!s_is_equal(6, result2, input_origin_v, error))std::cout << s.type() << "::kinVel() inverse origin failed" << std::endl;
			if (!s_is_equal(6, result3, input_origin_a, error))std::cout << s.type() << "::kinAcc() inverse origin failed" << std::endl;
			if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_origin_mfs, 1e-9))std::cout << s.type() << "::dynFce() inverse origin failed" << std::endl;


			// set ee status //
			m.generalMotionPool().at(0).setMpm(output_pm);
			m.generalMotionPool().at(0).setMva(output_va);
			m.generalMotionPool().at(0).setMaa(output_aa);
			// compute //
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
			s.dynAccAndFce();

			// get result //
			for (aris::Size i = 0; i < 6; ++i)
			{
				m.motionPool().at(i).updMp();
				m.motionPool().at(i).updMv();
				m.motionPool().at(i).updMa();
				result1[i] = m.motionPool().at(i).mp();
				result2[i] = m.motionPool().at(i).mv();
				result3[i] = m.motionPool().at(i).ma();
			}
			// check //
			if (!s_is_equal(6, result1, input_p, error))std::cout << s.type() << "::kinPos() inverse failed" << std::endl;
			if (!s_is_equal(6, result2, input_v, error))std::cout << s.type() << "::kinVel() inverse failed" << std::endl;
			if (!s_is_equal(6, result3, input_a, error))std::cout << s.type() << "::kinAcc() inverse failed" << std::endl;
			if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_mfs, 1e-9))std::cout << s.type() << "::dynFce() inverse failed" << std::endl;
		};	
		auto bench_pos_forward = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(true);
			m.generalMotionPool().at(0).activate(false);
			s.allocateMemory();
			std::cout << s.type() << "::forward computational pos time:" << aris::core::benchmark(bench_count, [&]()
			{
				if (count % 2)for (int i{ 0 }; i < 6; ++i) m.motionPool().at(i).setMp(input_p[i]);
				else for (int i{ 0 }; i < 6; ++i) m.motionPool().at(i).setMp(input_origin_p[i]);

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
			for (aris::Size i = 0; i < 6; ++i)
			{
				m.motionPool().at(i).setMp(input_p[i]);
				m.motionPool().at(i).setMv(input_v[i]);
				m.motionPool().at(i).setMa(input_a[i]);
			}
			s.allocateMemory();
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
			for (aris::Size i = 0; i < 6; ++i)
			{
				m.motionPool().at(i).setMp(input_p[i]);
				m.motionPool().at(i).setMv(input_v[i]);
				m.motionPool().at(i).setMa(input_a[i]);
			}
			s.allocateMemory();
			s.kinPos();
			s.kinVel();
			s.dynAccAndFce();

			std::cout << s.type() << "::forward computational dyn time:" << aris::core::benchmark(bench_count, [&]() {s.dynAccAndFce(); }) << std::endl;
		};
		auto bench_pos_inverse = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(false);
			m.generalMotionPool().at(0).activate(true);
			s.allocateMemory();
			std::cout << s.type() << "::inverse computational pos time:" << aris::core::benchmark(bench_count, [&]()
			{
				if (count % 2)for (int i{ 0 }; i < 6; ++i) m.generalMotionPool().at(0).setMpm(output_pm);
				else for (int i{ 0 }; i < 6; ++i) m.generalMotionPool().at(0).setMpm(output_origin_pm);

				// compute //
				s.kinPos();
				for (aris::Size i = 0; i < 6; ++i) { m.motionPool().at(i).updMp(); result1[i] = m.motionPool().at(i).mp(); }

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
			s.allocateMemory();
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
			s.allocateMemory();
			s.kinPos();
			s.kinVel();
			s.dynAccAndFce();
			std::cout << s.type() << "::inverse computational dyn time:" << aris::core::benchmark(bench_count, [&]() {s.dynAccAndFce(); }) << std::endl;
		};


		auto &gs = static_cast<LltGroundDividedSolver&>(*m.solverPool().findByName("gs"));
		auto &ps = static_cast<LltPartDividedSolver&>(*m.solverPool().findByName("ps"));
		auto &ds = static_cast<DiagSolver&>(*m.solverPool().findByName("ds"));
		

		std::cout << "test 6R robot:" << std::endl;

		auto plan = [&](const aris::dynamic::PlanParam& param)->int
		{
			double pm[16];
			std::copy(output_pm, output_pm + 16, pm);
			pm[3] += 0.01 * std::sin(2 * PI*param.count_ / 1000);
			param.model_->generalMotionPool().at(0).setMpm(pm);
			param.model_->setTime(param.count_ * 0.001);
			return 1000 - param.count_;
		};
		for (auto &mot : m.motionPool())mot.activate(false);
		m.generalMotionPool().at(0).activate(true);
		auto &s = m.simulatorPool().add<SolverSimulator>("simulator", ds);
		auto &r = m.simResultPool().add<SimResult>("result1");

		s.simulate(plan, 0, r);
		m.saveXml("C:\\Users\\py033\\Desktop\\m3.xml");
		
		auto &adams_simulator = m.simulatorPool().add<AdamsSimulator>("adams_simulator", ds);
		adams_simulator.saveAdams("C:\\Users\\py033\\Desktop\\m3.cmd", r);

		
		auto &gcs = m.solverPool().add<GroundCombineSolver>("gcs");
		gcs.setMaxError(1e-14);
		test_forward(gcs);
		test_inverse(gcs);
		bench_pos_inverse(gcs, 1000);
		bench_vel_inverse(gcs, 1000);
		bench_dyn_inverse(gcs, 1000);
		bench_pos_forward(gcs, 1000);
		bench_vel_forward(gcs, 1000);
		bench_dyn_forward(gcs, 1000);

		gs.setMaxError(1e-14);
		test_forward(gs);
		test_inverse(gs);
		bench_pos_inverse(gs, 1000);
		bench_vel_inverse(gs, 1000);
		bench_dyn_inverse(gs, 1000);
		bench_pos_forward(gs, 1000);
		bench_vel_forward(gs, 1000);
		bench_dyn_forward(gs, 1000);

		ps.setMaxError(1e-14);
		test_forward(ps);
		test_inverse(ps);
		bench_pos_inverse(ps, 1000);
		bench_vel_inverse(ps, 1000);
		bench_dyn_inverse(ps, 1000);
		bench_pos_forward(ps, 1000);
		bench_vel_forward(ps, 1000);
		bench_dyn_forward(ps, 1000);

		ds.setMaxError(1e-14);
		test_forward(ds);
		test_inverse(ds);
		bench_pos_inverse(ds, 1000);
		bench_vel_inverse(ds, 1000);
		bench_dyn_inverse(ds, 1000);
		bench_pos_forward(ds, 1000);
		bench_vel_forward(ds, 1000);
		bench_dyn_forward(ds, 1000);
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
		const double input_origin_mf[6]{ -13.3849595750926, -13.3927097990384, -13.3901587863615, -13.3901587863617, -13.3927097990384, -13.3849595750926 };
		const double output_origin_pm[16]{ 1,0,0,0,
			0, 0.999999999751072,2.2312668404904e-05,1.7078344386197,
			0, -2.23126684049141e-05,0.999999999751072,0.577658198650165,
			0,0,0,1 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ 0, -68.5999999829237, -0.00153064905217093, -0.0195999999954068,   0, 0 };

		const double input_p[6]{ 2.15,2.03,1.98,1.68,2.22,2.01 };
		const double input_v[6]{ 0.687,1.521,-0.325,0.665,1.225,-0.999 };
		const double input_a[6]{ 1.687,0.521,-1.325,1.665,0.225,-1.999 };
		const double input_mf[6]{ -54.3666620596338, -25.0817857667786, -18.1840142074996,   36.630055076576, -65.6950872736471,   75.5093067608351 };
		const double output_pm[16]{ 0.654617242227831, -0.16813527373803,0.737025641279234,0.0674004103296998,
			0.286892301165042,0.957269694021347, -0.0364354283699648,1.66351811346172,
			-0.699406229390514,0.235298241883176,0.674881962758251,0.907546391448817,
			0,0,0,1 };
		const double output_va[6]{ -1.67602445813444,0.322144550146041,1.43386389933679, -4.13258637478856,0.229701802785213,2.06026880988191 };
		const double output_aa[6]{ -3.99625983193204, -4.52459258496676,3.82662285536541, -4.70386456087171,10.2271223856012,12.7760010719168 };
		const double output_mfs[6]{ 43.6438945057721, -43.4082817089241,   0.842045165182085,   11.2174553475889, -25.1497728624644, -4.77548370421841 };

		double result1[16], result2[16], result3[16], result4[16];

		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_file_stewart);
		Model m;
		m.loadXml(xml_doc);

		auto &gs = static_cast<GroundDividedSolver&>(*m.solverPool().findByName("gs"));
		auto &ps = static_cast<PartDividedSolver&>(*m.solverPool().findByName("ps"));
		auto &ds = static_cast<DiagSolver&>(*m.solverPool().findByName("ds"));

		// test kinematic and dynamic model //
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
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
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
			if (!s_is_equal(6, result4, input_origin_mf, 1e-8))std::cout << s.type() << "::dynFce() forward origin failed" << std::endl;

			// set input //
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)
			{
				m.motionPool().at(i).setMp(input_p[i]);
				m.motionPool().at(i).setMv(input_v[i]);
				m.motionPool().at(i).setMa(input_a[i]);
			}
			// compute //
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
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
			if (!s_is_equal(6, result4, input_mf, 1e-8))std::cout << s.type() << "::dynFce() forward failed" << std::endl;

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
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
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
			if (!s_is_equal(6, result1, input_origin_p, error))std::cout << s.type() << "::kinPos() inverse origin failed" << std::endl;
			if (!s_is_equal(6, result2, input_origin_v, error))std::cout << s.type() << "::kinVel() inverse origin failed" << std::endl;
			if (!s_is_equal(6, result3, input_origin_a, error))std::cout << s.type() << "::kinAcc() inverse origin failed" << std::endl;
			if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_origin_mfs, 1e-8))std::cout << s.type() << "::dynFce() inverse origin failed" << std::endl;

			// set ee status //
			m.generalMotionPool().at(0).setMpm(output_pm);
			m.generalMotionPool().at(0).setMva(output_va);
			m.generalMotionPool().at(0).setMaa(output_aa);
			// compute //
			s.allocateMemory();
			s.kinPos();
			s.allocateMemory();
			s.kinVel();
			s.allocateMemory();
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
			if (!s_is_equal(6, result1, input_p, error))std::cout << s.type() << "::kinPos() inverse failed" << std::endl;
			if (!s_is_equal(6, result2, input_v, error))std::cout << s.type() << "::kinVel() inverse failed" << std::endl;
			if (!s_is_equal(6, result3, input_a, error))std::cout << s.type() << "::kinAcc() inverse failed" << std::endl;
			if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_mfs, 1e-8))std::cout << s.type() << "::dynFce() inverse failed" << std::endl;
		};
		auto bench_pos_forward = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(true);
			m.generalMotionPool().at(0).activate(false);
			s.allocateMemory();
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
			s.allocateMemory();
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
			s.allocateMemory();
			s.kinPos();
			s.kinVel();
			s.dynAccAndFce();

			std::cout << s.type() << "::forward computational dyn time:" << aris::core::benchmark(bench_count, [&]() {s.dynAccAndFce(); }) << std::endl;
		};
		auto bench_pos_inverse = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(false);
			m.generalMotionPool().at(0).activate(true);
			s.allocateMemory();
			std::cout << s.type() << "::inverse computational pos time:" << aris::core::benchmark(bench_count, [&]()
			{
				if (count % 2)for (aris::Size i{ 0 }; i < m.motionPool().size(); ++i) m.generalMotionPool().at(0).setMpm(output_pm);
				else for (aris::Size i{ 0 }; i < m.motionPool().size(); ++i) m.generalMotionPool().at(0).setMpm(output_origin_pm);

				// compute //
				s.kinPos();
				for (aris::Size i = 0; i < m.motionPool().size(); ++i){	m.motionPool().at(i).updMp(); result1[i] = m.motionPool().at(i).mp(); }

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
			s.allocateMemory();
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
			s.allocateMemory();
			s.kinPos();
			s.kinVel();
			s.dynAccAndFce();
			std::cout << s.type() << "::inverse computational dyn time:" << aris::core::benchmark(bench_count, [&]() {s.dynAccAndFce(); }) << std::endl;
		};

		gs.setMaxError(1e-10);
		ps.setMaxError(1e-10);
		ds.setMaxError(1e-10);

		std::cout << "test stewart robot:" << std::endl;

		auto &r = m.simResultPool().add<SimResult>("result1");
		auto &adams_simulator = m.simulatorPool().add<AdamsSimulator>("adams_simulator", ds);
		r.record();
		adams_simulator.saveAdams("C:\\Users\\py033\\Desktop\\m4.cmd", r, 0);


		auto &gcs = m.solverPool().add<GroundCombineSolver>("gcs");
		gcs.setMaxError(1e-14);
		test_forward(gcs);
		test_inverse(gcs);
		bench_pos_inverse(gcs, 100);
		bench_vel_inverse(gcs, 100);
		bench_dyn_inverse(gcs, 100);
		bench_pos_forward(gcs, 100);
		bench_vel_forward(gcs, 100);
		bench_dyn_forward(gcs, 100);

		gs.setMaxError(1e-14);
		test_forward(gs);
		test_inverse(gs);
		bench_pos_inverse(gs, 1000);
		bench_vel_inverse(gs, 1000);
		bench_dyn_inverse(gs, 1000);
		bench_pos_forward(gs, 1000);
		bench_vel_forward(gs, 1000);
		bench_dyn_forward(gs, 1000);

		ds.setMaxError(1e-14);
		test_forward(ds);
		test_inverse(ds);
		bench_pos_inverse(ds, 1000);
		bench_vel_inverse(ds, 1000);
		bench_dyn_inverse(ds, 1000);
		bench_pos_forward(ds, 1000);
		bench_vel_forward(ds, 1000);
		bench_dyn_forward(ds, 1000);

		ps.setMaxError(1e-14);
		test_forward(ps);
		test_inverse(ps);
		bench_pos_inverse(ps, 1000);
		bench_vel_inverse(ps, 1000);
		bench_dyn_inverse(ps, 1000);
		bench_pos_forward(ps, 1000);
		bench_vel_forward(ps, 1000);
		bench_dyn_forward(ps, 1000);




	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}

void test_model()
{
	std::cout << std::endl << "-----------------test model---------------------" << std::endl;
	test_part();
	test_constraint();
	test_solver_compute();
	test_solver_3R();
	test_solver_6R();
	test_solver_stewart();
	std::cout << "-----------------test model finished------------" << std::endl << std::endl;
}

