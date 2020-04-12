#include "test_dynamic_model_solver.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>
#include <aris/core/msg.hpp>

#include<type_traits>

using namespace aris::dynamic;

const char xml_file_ur5[] =
"<Model name=\"ur5\" time=\"0\">"
"    <Environment name=\"environment\" gravity=\"{0 , 0 , -9.8 , 0 , 0 , 0}\"/>"
"    <PartPoolElement name=\"part_pool\">"
"        <Part name=\"ground\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_0_j\" active=\"true\" pe=\"{0 , 0 , 0.089159 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                <Marker name=\"ee_makJ\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0 , 0 , 0.003 , 5.49778714108979 , 1.5707963267949 , 3.14159265358979}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/ground.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L1\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{3.7 , 0 , 0 , 0.3298883 , 0.0396800068327 , 0.0396800068327 , 0.00666 , 0 , 0 , 0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_0_i\" active=\"true\" pe=\"{0 , 0 , 0.089159 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                <Marker name=\"joint_1_j\" active=\"true\" pe=\"{0 , 0.13585 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0 , 0 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 3.14159265358979}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L1.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L2\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{8.393 , 2.35004 , 1.14018905 , 0.748311487 , 0.236720786311933 , 0.951620579779433 , 1.0397965583525 , -0.319252934 , -0.20952721636 , -0.10165811550895}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_1_i\" active=\"true\" pe=\"{0 , 0.13585 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                <Marker name=\"joint_2_j\" active=\"true\" pe=\"{0.425 , 0.01615 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0 , 0.13585 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L2.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L3\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{2.275 , 1.535625 , 0.03674125 , 0.202836725 , 0.022773090751775 , 1.10407490812027 , 1.0865835597435 , -0.02480034375 , -0.136914789375 , -0.00327581310875}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_2_i\" active=\"true\" pe=\"{0.425 , 0.01615 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                <Marker name=\"joint_3_j\" active=\"true\" pe=\"{0.81725 , 0.01615 , 0.0891590000000001 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0.425 , 0.01615 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L3.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L4\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1.219 , 0.99622775 , 0.01968685 , 0.108684821 , 0.121180928114039 , 0.935030114174039 , 1.033905071315 , -0.0160890781625 , -0.0888226699622501 , -0.00175525985915}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_3_i\" active=\"true\" pe=\"{0.81725 , 0.01615 , 0.0891590000000001 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                <Marker name=\"joint_4_j\" active=\"true\" pe=\"{0.81725 , 0.10915 , 0.0891590000000001 , 5.49778714378214 , 3.14159265358979 , 0.785398163397448}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0.81725 , 0.10915 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 1.5707963267949}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L4.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L5\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1.219 , 0.99622775 , 0.13305385 , 0.108684821 , 0.135385813214039 , 0.935030114174039 , 1.048109956415 , -0.1087382589125 , -0.0888226699622501 , -0.01186294821215}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_4_i\" active=\"true\" pe=\"{0.81725 , 0.10915 , 0.0891590000000001 , 5.49778714378214 , 3.14159265358979 , 0.785398163397448}\"/>"
"                <Marker name=\"joint_5_j\" active=\"true\" pe=\"{0.81725 , 0.10915 , -0.00549099999999994 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0.81725 , 0.10915 , -0.005491 , 3.14159265358979 , 1.5707963267949 , 6.28318530717959}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L5.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L6\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{0.1879 , 0.153561275 , 0.020509285 , -0.00103175889999999 , 0.0193807269912699 , 0.14264009052727 , 0.1615585404515 , -0.01676121316625 , 0.000843204961024989 , 0.000112616483934999}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_5_i\" active=\"true\" pe=\"{0.81725 , 0.10915 , -0.00549099999999994 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                <Marker name=\"ee_makI\" active=\"true\" pe=\"{0.81725 , 0.19145 , -0.00549099999999994 , 3.14159265358979 , 1.5707963267949 , 6.28318530717959}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0.81725 , 0.10915 , -0.005491 , 3.14159265358979 , 1.5707963267949 , 3.14159265358979}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L6.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"    </PartPoolElement>"
"    <JointPoolElement name=\"joint_pool\">"
"        <RevoluteJoint name=\"joint_0\" active=\"true\" prt_m=\"L1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_1\" active=\"true\" prt_m=\"L2\" prt_n=\"L1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_2\" active=\"true\" prt_m=\"L3\" prt_n=\"L2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_3\" active=\"true\" prt_m=\"L4\" prt_n=\"L3\" mak_i=\"joint_3_i\" mak_j=\"joint_3_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_4\" active=\"true\" prt_m=\"L5\" prt_n=\"L4\" mak_i=\"joint_4_i\" mak_j=\"joint_4_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_5\" active=\"true\" prt_m=\"L6\" prt_n=\"L5\" mak_i=\"joint_5_i\" mak_j=\"joint_5_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"    </JointPoolElement>"
"    <MotionPoolElement name=\"motion_pool\">"
"        <Motion name=\"motion_0\" active=\"true\" prt_m=\"L1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_1\" active=\"true\" prt_m=\"L2\" prt_n=\"L1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_2\" active=\"true\" prt_m=\"L3\" prt_n=\"L2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_3\" active=\"true\" prt_m=\"L4\" prt_n=\"L3\" mak_i=\"joint_3_i\" mak_j=\"joint_3_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_4\" active=\"true\" prt_m=\"L5\" prt_n=\"L4\" mak_i=\"joint_4_i\" mak_j=\"joint_4_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_5\" active=\"true\" prt_m=\"L6\" prt_n=\"L5\" mak_i=\"joint_5_i\" mak_j=\"joint_5_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"    </MotionPoolElement>"
"    <GeneralMotionPoolElement name=\"general_motion_pool\">"
"        <GeneralMotion name=\"ee\" active=\"false\" prt_m=\"L6\" prt_n=\"ground\" mak_i=\"ee_makI\" mak_j=\"ee_makJ\" cf=\"{0 , 0 , 0 , 0 , 0 , 0}\"/>"
"    </GeneralMotionPoolElement>"
"	 <ForcePoolElement name=\"force_pool\">"
"		 <SingleComponentForce name=\"F1\" active=\"true\" prt_m=\"L1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"F2\" active=\"true\" prt_m=\"L2\" prt_n=\"L1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"F3\" active=\"true\" prt_m=\"L3\" prt_n=\"L2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"F4\" active=\"true\" prt_m=\"L4\" prt_n=\"L3\" mak_i=\"joint_3_i\" mak_j=\"joint_3_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"F5\" active=\"true\" prt_m=\"L5\" prt_n=\"L4\" mak_i=\"joint_4_i\" mak_j=\"joint_4_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"F6\" active=\"true\" prt_m=\"L6\" prt_n=\"L5\" mak_i=\"joint_5_i\" mak_j=\"joint_5_j\" component=\"5\"/>"
"	 </ForcePoolElement>"
"    <SolverPoolElement name=\"solver_pool\">"
"        <UniversalSolver name=\"us\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"    </SolverPoolElement>"
"    <SimulatorPoolElement name=\"simulator_pool\"/>"
"    <SimResultPoolElement name=\"sim_result_pool\"/>"
"    <CalibratorPoolElement name=\"calibrator_pool\"/>"
"</Model>";

const char xml_file_stewart[] =
"<Model>"
"    <Environment name=\"environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"    <VariablePoolElement name=\"variable_pool\">"
"    </VariablePoolElement>"
"    <PartPoolElement name=\"part_pool\">"
"        <Part name=\"p1a\" active=\"true\" inertia=\"{1 , -0.62 , 0.13 , -0.58 , 105.0 , 116.25 , 100.28 , 20.11015 , 12.2000345614 , 0.58539}\" pe=\"{0.999999999999974 , 1.22522177619812e-16 , -9.28869564848867e-18 , 6.38378239159465e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u1i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p1j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p1b\" active=\"true\" inertia=\"{1 , -0.12 , 0.53 , -0.58 , 15.0 , 16.25 , 100.28 , 10.11015 , 12.2000345614 , 0.58539}\" pe=\"{0.0711481425892889 , 1.49999999999963 , 0.912443796234424 , 8.04911692853238e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p1i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s1j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p2a\" active=\"true\" inertia=\"{1 , -0.622 , 0.113 , -0.538 , 105.2 , 116.75 , 100.88 , 21.11015 , 11.2000345614 , 1.58539}\" pe=\"{0.999999999999995 , 1.22524189323061e-16 , -9.2876368573046e-18 , 5.55111512312578e-17 , 0.721024145526766 , 0.308719565228027}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u2i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p2j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p2b\" active=\"true\" inertia=\"{1 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{0.363127053316677 , 1.49999999999988 , 1.31832224563822 , 6.28318530717959 , 0.721024145526766 , 0.308719565228028}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p2i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s2j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p3a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{1.24902578429613e-16 , 3.6066466064807e-14 , 1.73199999999999 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u3i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p3j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p3b\" active=\"true\" inertia=\"{0.82156 , -0.6221 , 0.1312 , -0.5812 , 105.12 , 116.2512 , 100.2812 , 20.1101512 , 12.3 , 0.5853912}\" pe=\"{0.363127053316337 , 1.49999999999935 , 1.31832224563851 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p3i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s3j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p4a\" active=\"true\" inertia=\"{1.52156 , -0.521 , 0.231 , -0.481 , 115.1 , 106.251 , 110.281 , 21.110151 , 13.20003 , 0.555391}\" pe=\"{1.24898250620648e-16 , 1.52855080404276e-14 , 1.732 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u4i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p4j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p4b\" active=\"true\" inertia=\"{1.82756 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.134375029322252 , 1.49999999999964 , 1.36823895396183 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p4i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s4j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p5a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 112.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.999999999999993 , -1.0082029353865e-16 , 4.19175032725778e-17 , 6.28318530717959 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u5i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p5j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p5b\" active=\"true\" inertia=\"{1.82156 , -0.521 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.134375029322406 , 1.49999999999987 , 1.36823895396163 , 2.77555756156289e-17 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p5i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s5j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p6a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 125.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.999999999999969 , -1.00821934664985e-16 , 4.19165900651815e-17 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u6i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p6j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p6b\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 126.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{0.0711481425888235 , 1.49999999999959 , 0.912443796234401 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p6i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s6j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"up\" active=\"true\" inertia=\"{1.82156 , -0.651 , 0.1313 , -0.5814 , 105.1 , 116.271 , 100.221 , 20.120151 , 12.22003 , 0.583391}\" pe=\"{0.1 , 1.5 , 1.2 , 1.5707963267949 , 0.1 , 4.71238898038469}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"ee\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <Marker name=\"s1i\" pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                <Marker name=\"s2i\" pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s3i\" pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s4i\" pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s5i\" pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s6i\" pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\up.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"ground\" active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"origin\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <Marker name=\"u1o\" pe=\"{ 1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u2o\" pe=\"{ 1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u3o\" pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                <Marker name=\"u4o\" pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                <Marker name=\"u5o\" pe=\"{ -1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u6o\" pe=\"{ -1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u1j\" pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u2j\" pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u3j\" pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u4j\" pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u5j\" pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u6j\" pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"            </MarkerPoolElement>"
"        </Part>"
"    </PartPoolElement>"
"    <JointPoolElement name=\"joint_pool\">"
"        <UniversalJoint name=\"u1\" active=\"true\" prt_m=\"p1a\" prt_n=\"ground\" mak_i=\"u1i\" mak_j=\"u1j\"/>"
"        <PrismaticJoint name=\"p1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\"/>"
"        <SphericalJoint name=\"s1\" active=\"true\" prt_m=\"up\" prt_n=\"p1b\" mak_i=\"s1i\" mak_j=\"s1j\"/>"
"        <UniversalJoint name=\"u2\" active=\"true\" prt_m=\"p2a\" prt_n=\"ground\" mak_i=\"u2i\" mak_j=\"u2j\"/>"
"        <PrismaticJoint name=\"p2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\"/>"
"        <SphericalJoint name=\"s2\" active=\"true\" prt_m=\"up\" prt_n=\"p2b\" mak_i=\"s2i\" mak_j=\"s2j\"/>"
"        <UniversalJoint name=\"u3\" active=\"true\" prt_m=\"p3a\" prt_n=\"ground\" mak_i=\"u3i\" mak_j=\"u3j\"/>"
"        <PrismaticJoint name=\"p3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\"/>"
"        <SphericalJoint name=\"s3\" active=\"true\" prt_m=\"up\" prt_n=\"p3b\" mak_i=\"s3i\" mak_j=\"s3j\"/>"
"        <UniversalJoint name=\"u4\" active=\"true\" prt_m=\"p4a\" prt_n=\"ground\" mak_i=\"u4i\" mak_j=\"u4j\"/>"
"        <PrismaticJoint name=\"p4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\"/>"
"        <SphericalJoint name=\"s4\" active=\"true\" prt_m=\"up\" prt_n=\"p4b\" mak_i=\"s4i\" mak_j=\"s4j\"/>"
"        <UniversalJoint name=\"u5\" active=\"true\" prt_m=\"p5a\" prt_n=\"ground\" mak_i=\"u5i\" mak_j=\"u5j\"/>"
"        <PrismaticJoint name=\"p5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\"/>"
"        <SphericalJoint name=\"s5\" active=\"true\" prt_m=\"up\" prt_n=\"p5b\" mak_i=\"s5i\" mak_j=\"s5j\"/>"
"        <UniversalJoint name=\"u6\" active=\"true\" prt_m=\"p6a\" prt_n=\"ground\" mak_i=\"u6i\" mak_j=\"u6j\"/>"
"        <PrismaticJoint name=\"p6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\"/>"
"        <SphericalJoint name=\"s6\" active=\"true\" prt_m=\"up\" prt_n=\"p6b\" mak_i=\"s6i\" mak_j=\"s6j\"/>"
"    </JointPoolElement>"
"    <MotionPoolElement name=\"motion_pool\">"
"        <Motion name=\"m1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" frc_coe=\"{0, 0, 0}\" component=\"2\"/>"
"        <Motion name=\"m2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" frc_coe=\"{0, 0, 0}\" component=\"2\"/>"
"        <Motion name=\"m3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" frc_coe=\"{0, 0, 0}\" component=\"2\"/>"
"        <Motion name=\"m4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" frc_coe=\"{0, 0, 0}\" component=\"2\"/>"
"        <Motion name=\"m5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" frc_coe=\"{0, 0, 0}\" component=\"2\"/>"
"        <Motion name=\"m6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" frc_coe=\"{0, 0, 0}\" component=\"2\"/>"
"    </MotionPoolElement>"
"	<ForcePoolElement name=\"force_pool\">"
"		<SingleComponentForce name=\"f1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" component=\"2\"/>"
"	</ForcePoolElement>"
"    <GeneralMotionPoolElement name=\"general_motion_pool\">"
"        <GeneralMotion name=\"ee_mot\" active=\"false\" prt_m=\"up\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"    </GeneralMotionPoolElement>"
"    <SolverPoolElement name=\"solver_pool\">"
"        <UniversalSolver name=\"us\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"    </SolverPoolElement>"
"</Model>";

const char xml_file_ur5_on_stewart[] =
"<Model>"
"    <Environment name=\"environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"    <VariablePoolElement name=\"variable_pool\">"
"        <MatrixVariable name=\"PI\">3.14159265358979</MatrixVariable>"
"        <MatrixVariable name=\"Mot_friction\">{0, 0, 0}</MatrixVariable>"
"    </VariablePoolElement>"
"    <PartPoolElement name=\"part_pool\">"
"        <Part name=\"L1\" active=\"true\" pe=\"{0.1 , 1.5 , 1.2 , 0.1 , PI/2 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{3.7 , 0 , 0 , 0.3298883 , 0.0396800068327 , 0.0396800068327 , 0.00666 , 0 , 0 , 0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_0_i\" active=\"true\" pe=\"{0 , 0 , 0.089159 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                <Marker name=\"joint_1_j\" active=\"true\" pe=\"{0 , 0.13585 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0 , 0 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 3.14159265358979}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L1.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L2\" active=\"true\" pe=\"{0.1 , 1.5 , 1.2 , 0.1 , PI/2 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{8.393 , 2.35004 , 1.14018905 , 0.748311487 , 0.236720786311933 , 0.951620579779433 , 1.0397965583525 , -0.319252934 , -0.20952721636 , -0.10165811550895}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_1_i\" active=\"true\" pe=\"{0 , 0.13585 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                <Marker name=\"joint_2_j\" active=\"true\" pe=\"{0.425 , 0.01615 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0 , 0.13585 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L2.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L3\" active=\"true\" pe=\"{0.1 , 1.5 , 1.2 , 0.1 , PI/2 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{2.275 , 1.535625 , 0.03674125 , 0.202836725 , 0.022773090751775 , 1.10407490812027 , 1.0865835597435 , -0.02480034375 , -0.136914789375 , -0.00327581310875}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_2_i\" active=\"true\" pe=\"{0.425 , 0.01615 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                <Marker name=\"joint_3_j\" active=\"true\" pe=\"{0.81725 , 0.01615 , 0.0891590000000001 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0.425 , 0.01615 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L3.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L4\" active=\"true\" pe=\"{0.1 , 1.5 , 1.2 , 0.1 , PI/2 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1.219 , 0.99622775 , 0.01968685 , 0.108684821 , 0.121180928114039 , 0.935030114174039 , 1.033905071315 , -0.0160890781625 , -0.0888226699622501 , -0.00175525985915}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_3_i\" active=\"true\" pe=\"{0.81725 , 0.01615 , 0.0891590000000001 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                <Marker name=\"joint_4_j\" active=\"true\" pe=\"{0.81725 , 0.10915 , 0.0891590000000001 , 5.49778714378214 , 3.14159265358979 , 0.785398163397448}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0.81725 , 0.10915 , 0.089159 , 3.14159265358979 , 1.5707963267949 , 1.5707963267949}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L4.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L5\" active=\"true\" pe=\"{0.1 , 1.5 , 1.2 , 0.1 , PI/2 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1.219 , 0.99622775 , 0.13305385 , 0.108684821 , 0.135385813214039 , 0.935030114174039 , 1.048109956415 , -0.1087382589125 , -0.0888226699622501 , -0.01186294821215}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_4_i\" active=\"true\" pe=\"{0.81725 , 0.10915 , 0.0891590000000001 , 5.49778714378214 , 3.14159265358979 , 0.785398163397448}\"/>"
"                <Marker name=\"joint_5_j\" active=\"true\" pe=\"{0.81725 , 0.10915 , -0.00549099999999994 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0.81725 , 0.10915 , -0.005491 , 3.14159265358979 , 1.5707963267949 , 6.28318530717959}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L5.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"L6\" active=\"true\" pe=\"{0.1 , 1.5 , 1.2 , 0.1 , PI/2 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{0.1879 , 0.153561275 , 0.020509285 , -0.00103175889999999 , 0.0193807269912699 , 0.14264009052727 , 0.1615585404515 , -0.01676121316625 , 0.000843204961024989 , 0.000112616483934999}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_5_i\" active=\"true\" pe=\"{0.81725 , 0.10915 , -0.00549099999999994 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                <Marker name=\"ee_makI\" active=\"true\" pe=\"{0.81725 , 0.19145 , -0.00549099999999994 , 3.14159265358979 , 1.5707963267949 , 6.28318530717959}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <FileGeometry name=\"file_geometry\" pe=\"{0.81725 , 0.10915 , -0.005491 , 3.14159265358979 , 1.5707963267949 , 3.14159265358979}\" graphic_file_path=\"C:/aris/resource/aris_robot/ur5/L6.DWG\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p1a\" active=\"true\" inertia=\"{1 , -0.62 , 0.13 , -0.58 , 105.0 , 116.25 , 100.28 , 20.11015 , 12.2000345614 , 0.58539}\" pe=\"{0.999999999999974 , 1.22522177619812e-16 , -9.28869564848867e-18 , 6.38378239159465e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u1i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p1j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p1b\" active=\"true\" inertia=\"{1 , -0.12 , 0.53 , -0.58 , 15.0 , 16.25 , 100.28 , 10.11015 , 12.2000345614 , 0.58539}\" pe=\"{0.0711481425892889 , 1.49999999999963 , 0.912443796234424 , 8.04911692853238e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p1i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s1j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p2a\" active=\"true\" inertia=\"{1 , -0.622 , 0.113 , -0.538 , 105.2 , 116.75 , 100.88 , 21.11015 , 11.2000345614 , 1.58539}\" pe=\"{0.999999999999995 , 1.22524189323061e-16 , -9.2876368573046e-18 , 5.55111512312578e-17 , 0.721024145526766 , 0.308719565228027}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u2i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p2j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p2b\" active=\"true\" inertia=\"{1 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{0.363127053316677 , 1.49999999999988 , 1.31832224563822 , 6.28318530717959 , 0.721024145526766 , 0.308719565228028}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p2i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s2j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p3a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{1.24902578429613e-16 , 3.6066466064807e-14 , 1.73199999999999 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u3i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p3j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p3b\" active=\"true\" inertia=\"{0.82156 , -0.6221 , 0.1312 , -0.5812 , 105.12 , 116.2512 , 100.2812 , 20.1101512 , 12.3 , 0.5853912}\" pe=\"{0.363127053316337 , 1.49999999999935 , 1.31832224563851 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p3i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s3j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p4a\" active=\"true\" inertia=\"{1.52156 , -0.521 , 0.231 , -0.481 , 115.1 , 106.251 , 110.281 , 21.110151 , 13.20003 , 0.555391}\" pe=\"{1.24898250620648e-16 , 1.52855080404276e-14 , 1.732 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u4i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p4j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p4b\" active=\"true\" inertia=\"{1.82756 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.134375029322252 , 1.49999999999964 , 1.36823895396183 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p4i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s4j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p5a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 112.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.999999999999993 , -1.0082029353865e-16 , 4.19175032725778e-17 , 6.28318530717959 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u5i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p5j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p5b\" active=\"true\" inertia=\"{1.82156 , -0.521 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.134375029322406 , 1.49999999999987 , 1.36823895396163 , 2.77555756156289e-17 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p5i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s5j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p6a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 125.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.999999999999969 , -1.00821934664985e-16 , 4.19165900651815e-17 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u6i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p6j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p6b\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 126.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{0.0711481425888235 , 1.49999999999959 , 0.912443796234401 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p6i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s6j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"up\" active=\"true\" inertia=\"{1.82156 , -0.651 , 0.1313 , -0.5814 , 105.1 , 116.271 , 100.221 , 20.120151 , 12.22003 , 0.583391}\" pe=\"{0.1 , 1.5 , 1.2 , 1.5707963267949 , 0.1 , 4.71238898038469}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"ee\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <Marker name=\"s1i\" pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                <Marker name=\"s2i\" pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s3i\" pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s4i\" pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s5i\" pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s6i\" pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                <Marker name=\"joint_0_j\" active=\"true\" pe=\"{0 , 0.089159+0.025 , 0 , 0, -PI/2, PI/2}\"/>"
"                <Marker name=\"ee_makJ\" active=\"true\" pe=\"{0 , 0+0.025 , 0 , 0.0, -PI/2, 0.0}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\up.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"ground\" active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"origin\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <Marker name=\"u1o\" pe=\"{ 1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u2o\" pe=\"{ 1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u3o\" pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                <Marker name=\"u4o\" pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                <Marker name=\"u5o\" pe=\"{ -1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u6o\" pe=\"{ -1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u1j\" pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u2j\" pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u3j\" pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u4j\" pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u5j\" pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u6j\" pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"            </MarkerPoolElement>"
"        </Part>"
"    </PartPoolElement>"
"    <JointPoolElement name=\"joint_pool\">"
"        <RevoluteJoint name=\"joint_0\" active=\"true\" prt_m=\"L1\" prt_n=\"up\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_1\" active=\"true\" prt_m=\"L2\" prt_n=\"L1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_2\" active=\"true\" prt_m=\"L3\" prt_n=\"L2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_3\" active=\"true\" prt_m=\"L4\" prt_n=\"L3\" mak_i=\"joint_3_i\" mak_j=\"joint_3_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_4\" active=\"true\" prt_m=\"L5\" prt_n=\"L4\" mak_i=\"joint_4_i\" mak_j=\"joint_4_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_5\" active=\"true\" prt_m=\"L6\" prt_n=\"L5\" mak_i=\"joint_5_i\" mak_j=\"joint_5_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <UniversalJoint name=\"u1\" active=\"true\" prt_m=\"p1a\" prt_n=\"ground\" mak_i=\"u1i\" mak_j=\"u1j\"/>"
"        <PrismaticJoint name=\"p1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\"/>"
"        <SphericalJoint name=\"s1\" active=\"true\" prt_m=\"up\" prt_n=\"p1b\" mak_i=\"s1i\" mak_j=\"s1j\"/>"
"        <UniversalJoint name=\"u2\" active=\"true\" prt_m=\"p2a\" prt_n=\"ground\" mak_i=\"u2i\" mak_j=\"u2j\"/>"
"        <PrismaticJoint name=\"p2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\"/>"
"        <SphericalJoint name=\"s2\" active=\"true\" prt_m=\"up\" prt_n=\"p2b\" mak_i=\"s2i\" mak_j=\"s2j\"/>"
"        <UniversalJoint name=\"u3\" active=\"true\" prt_m=\"p3a\" prt_n=\"ground\" mak_i=\"u3i\" mak_j=\"u3j\"/>"
"        <PrismaticJoint name=\"p3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\"/>"
"        <SphericalJoint name=\"s3\" active=\"true\" prt_m=\"up\" prt_n=\"p3b\" mak_i=\"s3i\" mak_j=\"s3j\"/>"
"        <UniversalJoint name=\"u4\" active=\"true\" prt_m=\"p4a\" prt_n=\"ground\" mak_i=\"u4i\" mak_j=\"u4j\"/>"
"        <PrismaticJoint name=\"p4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\"/>"
"        <SphericalJoint name=\"s4\" active=\"true\" prt_m=\"up\" prt_n=\"p4b\" mak_i=\"s4i\" mak_j=\"s4j\"/>"
"        <UniversalJoint name=\"u5\" active=\"true\" prt_m=\"p5a\" prt_n=\"ground\" mak_i=\"u5i\" mak_j=\"u5j\"/>"
"        <PrismaticJoint name=\"p5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\"/>"
"        <SphericalJoint name=\"s5\" active=\"true\" prt_m=\"up\" prt_n=\"p5b\" mak_i=\"s5i\" mak_j=\"s5j\"/>"
"        <UniversalJoint name=\"u6\" active=\"true\" prt_m=\"p6a\" prt_n=\"ground\" mak_i=\"u6i\" mak_j=\"u6j\"/>"
"        <PrismaticJoint name=\"p6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\"/>"
"        <SphericalJoint name=\"s6\" active=\"true\" prt_m=\"up\" prt_n=\"p6b\" mak_i=\"s6i\" mak_j=\"s6j\"/>"
"    </JointPoolElement>"
"    <MotionPoolElement name=\"motion_pool\">"
"        <Motion name=\"motion_0\" active=\"true\" prt_m=\"L1\" prt_n=\"up\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_1\" active=\"true\" prt_m=\"L2\" prt_n=\"L1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_2\" active=\"true\" prt_m=\"L3\" prt_n=\"L2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_3\" active=\"true\" prt_m=\"L4\" prt_n=\"L3\" mak_i=\"joint_3_i\" mak_j=\"joint_3_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_4\" active=\"true\" prt_m=\"L5\" prt_n=\"L4\" mak_i=\"joint_4_i\" mak_j=\"joint_4_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_5\" active=\"true\" prt_m=\"L6\" prt_n=\"L5\" mak_i=\"joint_5_i\" mak_j=\"joint_5_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"m1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"    </MotionPoolElement>"
"	 <ForcePoolElement name=\"force_pool\">"
"		 <SingleComponentForce name=\"FF1\" active=\"true\" prt_m=\"L1\" prt_n=\"up\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"FF2\" active=\"true\" prt_m=\"L2\" prt_n=\"L1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"FF3\" active=\"true\" prt_m=\"L3\" prt_n=\"L2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"FF4\" active=\"true\" prt_m=\"L4\" prt_n=\"L3\" mak_i=\"joint_3_i\" mak_j=\"joint_3_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"FF5\" active=\"true\" prt_m=\"L5\" prt_n=\"L4\" mak_i=\"joint_4_i\" mak_j=\"joint_4_j\" component=\"5\"/>"
"		 <SingleComponentForce name=\"FF6\" active=\"true\" prt_m=\"L6\" prt_n=\"L5\" mak_i=\"joint_5_i\" mak_j=\"joint_5_j\" component=\"5\"/>"
"		<SingleComponentForce name=\"f1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" component=\"2\"/>"
"	</ForcePoolElement>"
"    <GeneralMotionPoolElement name=\"general_motion_pool\">"
"        <GeneralMotion name=\"ee\" active=\"false\" prt_m=\"L6\" prt_n=\"up\" mak_i=\"ee_makI\" mak_j=\"ee_makJ\" cf=\"{0 , 0 , 0 , 0 , 0 , 0}\"/>"
"        <GeneralMotion name=\"ee_mot\" active=\"false\" prt_m=\"up\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"    </GeneralMotionPoolElement>"
"    <SolverPoolElement name=\"solver_pool\">"
"        <UniversalSolver name=\"us\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"    </SolverPoolElement>"
"</Model>";


const char xml_file_multi[] =
"<Model>"
"    <Environment name=\"environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"    <VariablePoolElement name=\"variable_pool\">"
"        <MatrixVariable name=\"PI\">3.14159265358979</MatrixVariable>"
"        <MatrixVariable name=\"Mot_friction\">{0, 0, 0}</MatrixVariable>"
"    </VariablePoolElement>"
"    <PartPoolElement name=\"part_pool\">"
"        <Part name=\"part_1\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_0_i\" active=\"true\" pe=\"{0 , 0 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                <Marker name=\"joint_1_j\" active=\"true\" pe=\"{1 , 0 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\"/>"
"        </Part>"
"        <Part name=\"part_2\" active=\"true\" pe=\"{1 , 0 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_1_i\" active=\"true\" pe=\"{0 , 0 , 0 , 3.06161699786838e-17 , 0 , 3.06161699786838e-17}\"/>"
"                <Marker name=\"joint_2_j\" active=\"true\" pe=\"{1 , 6.12323399573677e-17 , 0 , 3.06161699786838e-17 , 0 , 3.06161699786838e-17}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\"/>"
"        </Part>"
"        <Part name=\"part_3\" active=\"true\" pe=\"{1 , 1 , 0 , 1.5707963267949 , 0 , 1.5707963267949}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"joint_2_i\" active=\"true\" pe=\"{0 , 0 , 0 , 5.49778714378214 , 0 , 5.49778714378214}\"/>"
"                <Marker name=\"general_motion_0_i\" active=\"true\" pe=\"{1 , 1.22464679914735e-16 , 0 , -0 , 0 , -0}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\"/>"
"        </Part>"
"		<Part name=\"part0\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<MarkerPoolElement name=\"marker_pool\">"
"				<Marker name=\"R0i\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<Marker name=\"R1j\" active=\"true\" pe=\"{0.127581045131803 , -0.255551891814505 , 0.743785121868611 , 1.39801111882379 , 0.92874853281329 , 2.99910369947683}\"/>"
"			</MarkerPoolElement>"
"		</Part>"
"		<Part name=\"part1\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.83142459280181 , 1.5646920289751 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<MarkerPoolElement name=\"marker_pool\">"
"				<Marker name=\"R1i\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<Marker name=\"R2j\" active=\"true\" pe=\"{0 , 0 , 0 , 6.21426444075139 , 1.09203109440023 , 1.87858662805362}\"/>"
"			</MarkerPoolElement>"
"		</Part>"
"		<Part name=\"part2\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322217}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<MarkerPoolElement name=\"marker_pool\">"
"				<Marker name=\"R2i\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<Marker name=\"R3j\" active=\"true\" pe=\"{0 , -0.996734365258792 , 0.0807502638519182 , 1.5707963267949 , 7.66425882286152e-17 , 4.71238898038469}\"/>"
"			</MarkerPoolElement>"
"		</Part>"
"		<Part name=\"part3\" active=\"true\" pe=\"{1.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<MarkerPoolElement name=\"marker_pool\">"
"				<Marker name=\"R3i\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<Marker name=\"R4j\" active=\"true\" pe=\"{1.53280055883457 , 0.0630122434114775 , 0.777786541421676 , 4.71238898038469 , 1.11022302462516e-16 , 1.5707963267949}\"/>"
"			</MarkerPoolElement>"
"		</Part>"
"		<Part name=\"part4\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<MarkerPoolElement name=\"marker_pool\">"
"				<Marker name=\"R4i\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<Marker name=\"R5j\" active=\"true\" pe=\"{0 , 0 , 0 , 0.299854573312955 , 1.27790273658208 , 3.56732345865295}\"/>"
"			</MarkerPoolElement>"
"		</Part>"
"		<Part name=\"part5\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 1.928105009803 , 1.5084212451233 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<MarkerPoolElement name=\"marker_pool\">"
"				<Marker name=\"R5i\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<Marker name=\"end_effector\" active=\"true\" pe=\"{1.6840663068739 , 0.615533362716626 , -2.33680109411025 , -0 , 1.5084212451233 , 1.2134876437868}\"/>"
"			</MarkerPoolElement>"
"		</Part>"
"        <Part name=\"p1a\" active=\"true\" inertia=\"{1 , -0.62 , 0.13 , -0.58 , 105.0 , 116.25 , 100.28 , 20.11015 , 12.2000345614 , 0.58539}\" pe=\"{0.999999999999974 , 1.22522177619812e-16 , -9.28869564848867e-18 , 6.38378239159465e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u1i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p1j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p1b\" active=\"true\" inertia=\"{1 , -0.12 , 0.53 , -0.58 , 15.0 , 16.25 , 100.28 , 10.11015 , 12.2000345614 , 0.58539}\" pe=\"{0.0711481425892889 , 1.49999999999963 , 0.912443796234424 , 8.04911692853238e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p1i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s1j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p2a\" active=\"true\" inertia=\"{1 , -0.622 , 0.113 , -0.538 , 105.2 , 116.75 , 100.88 , 21.11015 , 11.2000345614 , 1.58539}\" pe=\"{0.999999999999995 , 1.22524189323061e-16 , -9.2876368573046e-18 , 5.55111512312578e-17 , 0.721024145526766 , 0.308719565228027}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u2i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p2j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p2b\" active=\"true\" inertia=\"{1 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{0.363127053316677 , 1.49999999999988 , 1.31832224563822 , 6.28318530717959 , 0.721024145526766 , 0.308719565228028}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p2i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s2j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p3a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{1.24902578429613e-16 , 3.6066466064807e-14 , 1.73199999999999 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u3i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p3j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p3b\" active=\"true\" inertia=\"{0.82156 , -0.6221 , 0.1312 , -0.5812 , 105.12 , 116.2512 , 100.2812 , 20.1101512 , 12.3 , 0.5853912}\" pe=\"{0.363127053316337 , 1.49999999999935 , 1.31832224563851 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p3i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s3j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p4a\" active=\"true\" inertia=\"{1.52156 , -0.521 , 0.231 , -0.481 , 115.1 , 106.251 , 110.281 , 21.110151 , 13.20003 , 0.555391}\" pe=\"{1.24898250620648e-16 , 1.52855080404276e-14 , 1.732 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u4i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p4j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p4b\" active=\"true\" inertia=\"{1.82756 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.134375029322252 , 1.49999999999964 , 1.36823895396183 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p4i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s4j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p5a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 112.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.999999999999993 , -1.0082029353865e-16 , 4.19175032725778e-17 , 6.28318530717959 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u5i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p5j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p5b\" active=\"true\" inertia=\"{1.82156 , -0.521 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.134375029322406 , 1.49999999999987 , 1.36823895396163 , 2.77555756156289e-17 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p5i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s5j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p6a\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 125.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.999999999999969 , -1.00821934664985e-16 , 4.19165900651815e-17 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"u6i\" pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <Marker name=\"p6j\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"p6b\" active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 126.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{0.0711481425888235 , 1.49999999999959 , 0.912443796234401 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"p6i\" pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <Marker name=\"s6j\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"up\" active=\"true\" inertia=\"{1.82156 , -0.651 , 0.1313 , -0.5814 , 105.1 , 116.271 , 100.221 , 20.120151 , 12.22003 , 0.583391}\" pe=\"{0.1 , 1.5 , 1.2 , 1.5707963267949 , 0.1 , 4.71238898038469}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"ee\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <Marker name=\"s1i\" pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                <Marker name=\"s2i\" pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s3i\" pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s4i\" pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s5i\" pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                <Marker name=\"s6i\" pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                <Marker name=\"joint_0_j\" active=\"true\" pe=\"{0 , 0.089159+0.025 , 0 , 0, -PI/2, PI/2}\"/>"
"                <Marker name=\"ee_makJ\" active=\"true\" pe=\"{0 , 0+0.025 , 0 , 0.0, -PI/2, 0.0}\"/>"
"            </MarkerPoolElement>"
"            <GeometryPoolElement name=\"geometry_pool\">"
"                <ParasolidGeometry name=\"solid\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\up.xmt_txt\"/>"
"            </GeometryPoolElement>"
"        </Part>"
"        <Part name=\"ground\" active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <MarkerPoolElement name=\"marker_pool\">"
"                <Marker name=\"origin\" pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <Marker name=\"u1o\" pe=\"{ 1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u2o\" pe=\"{ 1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u3o\" pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                <Marker name=\"u4o\" pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                <Marker name=\"u5o\" pe=\"{ -1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u6o\" pe=\"{ -1,0,0,0,0,0 }\"/>"
"                <Marker name=\"u1j\" pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u2j\" pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u3j\" pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u4j\" pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u5j\" pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <Marker name=\"u6j\" pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"				<Marker name=\"R0j\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\"/>"
"                <Marker name=\"joint_0_j\" active=\"true\" pe=\"{0 , 0 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                <Marker name=\"general_motion_0_j\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"            </MarkerPoolElement>"
"        </Part>"
"    </PartPoolElement>"
"    <JointPoolElement name=\"joint_pool\">"
"        <RevoluteJoint name=\"joint_0\" active=\"true\" prt_m=\"part_1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_1\" active=\"true\" prt_m=\"part_2\" prt_n=\"part_1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <RevoluteJoint name=\"joint_2\" active=\"true\" prt_m=\"part_3\" prt_n=\"part_2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"		<RevoluteJoint name=\"R0\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\"/>"
"		<RevoluteJoint name=\"R1\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\"/>"
"		<RevoluteJoint name=\"R2\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\"/>"
"		<RevoluteJoint name=\"R3\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\"/>"
"		<RevoluteJoint name=\"R4\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\"/>"
"		<RevoluteJoint name=\"R5\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\"/>"
"        <UniversalJoint name=\"u1\" active=\"true\" prt_m=\"p1a\" prt_n=\"ground\" mak_i=\"u1i\" mak_j=\"u1j\"/>"
"        <PrismaticJoint name=\"p1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\"/>"
"        <SphericalJoint name=\"s1\" active=\"true\" prt_m=\"up\" prt_n=\"p1b\" mak_i=\"s1i\" mak_j=\"s1j\"/>"
"        <UniversalJoint name=\"u2\" active=\"true\" prt_m=\"p2a\" prt_n=\"ground\" mak_i=\"u2i\" mak_j=\"u2j\"/>"
"        <PrismaticJoint name=\"p2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\"/>"
"        <SphericalJoint name=\"s2\" active=\"true\" prt_m=\"up\" prt_n=\"p2b\" mak_i=\"s2i\" mak_j=\"s2j\"/>"
"        <UniversalJoint name=\"u3\" active=\"true\" prt_m=\"p3a\" prt_n=\"ground\" mak_i=\"u3i\" mak_j=\"u3j\"/>"
"        <PrismaticJoint name=\"p3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\"/>"
"        <SphericalJoint name=\"s3\" active=\"true\" prt_m=\"up\" prt_n=\"p3b\" mak_i=\"s3i\" mak_j=\"s3j\"/>"
"        <UniversalJoint name=\"u4\" active=\"true\" prt_m=\"p4a\" prt_n=\"ground\" mak_i=\"u4i\" mak_j=\"u4j\"/>"
"        <PrismaticJoint name=\"p4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\"/>"
"        <SphericalJoint name=\"s4\" active=\"true\" prt_m=\"up\" prt_n=\"p4b\" mak_i=\"s4i\" mak_j=\"s4j\"/>"
"        <UniversalJoint name=\"u5\" active=\"true\" prt_m=\"p5a\" prt_n=\"ground\" mak_i=\"u5i\" mak_j=\"u5j\"/>"
"        <PrismaticJoint name=\"p5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\"/>"
"        <SphericalJoint name=\"s5\" active=\"true\" prt_m=\"up\" prt_n=\"p5b\" mak_i=\"s5i\" mak_j=\"s5j\"/>"
"        <UniversalJoint name=\"u6\" active=\"true\" prt_m=\"p6a\" prt_n=\"ground\" mak_i=\"u6i\" mak_j=\"u6j\"/>"
"        <PrismaticJoint name=\"p6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\"/>"
"        <SphericalJoint name=\"s6\" active=\"true\" prt_m=\"up\" prt_n=\"p6b\" mak_i=\"s6i\" mak_j=\"s6j\"/>"
"    </JointPoolElement>"
"    <MotionPoolElement name=\"motion_pool\">"
"        <Motion name=\"motion_0\" active=\"true\" prt_m=\"part_1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_1\" active=\"true\" prt_m=\"part_2\" prt_n=\"part_1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <Motion name=\"motion_2\" active=\"true\" prt_m=\"part_3\" prt_n=\"part_2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"		<Motion name=\"M0\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<Motion name=\"M1\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<Motion name=\"M2\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<Motion name=\"M3\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<Motion name=\"M4\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<Motion name=\"M5\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        <Motion name=\"m1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"        <Motion name=\"m6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" frc_coe=\"{0 , 0 , 0}\" component=\"2\"/>"
"    </MotionPoolElement>"
"	<ForcePoolElement name=\"force_pool\">"
"        <SingleComponentForce name=\"F3R1\" active=\"true\" prt_m=\"part_1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" component=\"5\"/>"
"        <SingleComponentForce name=\"F3R2\" active=\"true\" prt_m=\"part_2\" prt_n=\"part_1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" component=\"5\"/>"
"        <SingleComponentForce name=\"F3R3\" active=\"true\" prt_m=\"part_3\" prt_n=\"part_2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" component=\"5\"/>"
"		<SingleComponentForce name=\"F1\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" component=\"5\"/>"
"		<SingleComponentForce name=\"F2\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" component=\"5\"/>"
"		<SingleComponentForce name=\"F3\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" component=\"5\"/>"
"		<SingleComponentForce name=\"F4\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" component=\"5\"/>"
"		<SingleComponentForce name=\"F5\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" component=\"5\"/>"
"		<SingleComponentForce name=\"F6\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" component=\"5\"/>"
"		<SingleComponentForce name=\"f1\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f2\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f3\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f4\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f5\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" component=\"2\"/>"
"		<SingleComponentForce name=\"f6\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" component=\"2\"/>"
"	</ForcePoolElement>"
"    <GeneralMotionPoolElement name=\"general_motion_pool\">"
"        <GeneralMotion name=\"general_motion_0\" active=\"true\" prt_m=\"part_3\" prt_n=\"ground\" mak_i=\"general_motion_0_i\" mak_j=\"general_motion_0_j\" cf=\"{0 , 0 , 0 , 0 , 0 , 0}\"/>"
"		<GeneralMotion name=\"ee_mot\" active=\"true\" prt_m=\"part5\" prt_n=\"ground\" mak_i=\"end_effector\" mak_j=\"origin\"/>"
"        <GeneralMotion name=\"ee_mot\" active=\"false\" prt_m=\"up\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"    </GeneralMotionPoolElement>"
"    <SolverPoolElement name=\"solver_pool\">"
"        <UniversalSolver name=\"us\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"    </SolverPoolElement>"
"</Model>";


void test_solver(Model &m, const double *ipo, const double *ivo, const double *iao, const double *ifo, 
	const double *opo, const double *ovo, const double *oao, const double *ofo, 
	const double *ipt, const double *ivt, const double *iat, const double *ift, 
	const double *opt, const double *ovt, const double *oat, const double *oft, const double *error)
{
	// 测试运动学正解、反解、动力学正向正解、反解/反向正解、反解
	for (auto &s : m.solverPool())
	{
		/////////////////////////////////////////////////////正向，从输入到输出///////////////////////////////////////////////////
		std::vector<double> result_pool(std::max(16 * m.generalMotionPool().size(), m.motionPool().size()));
		std::vector<double> result2_pool(6 * m.generalMotionPool().size());
		std::vector<double> result3_pool(6 * m.generalMotionPool().size());
		std::vector<double> result4_pool(6 * m.generalMotionPool().size());
		
		double *result = result_pool.data();
		double *result2 = result2_pool.data();
		double *result3 = result3_pool.data();
		double *result4 = result4_pool.data();
		
		// set topology //
		for (auto &fce : m.forcePool())fce.activate(false);
		for (auto &mot : m.motionPool())mot.activate(true);
		for (auto &gm : m.generalMotionPool())gm.activate(false);
		// set origin //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).setMp(ipo[i]);
			m.motionPool().at(i).setMv(ivo[i]);
			m.motionPool().at(i).setMa(iao[i]);
		}
		// compute origin //
		m.init();
		s.kinPos();
		m.init();
		s.kinVel();
		m.init();
		s.dynAccAndFce();
		std::cout << "iter count:" << s.iterCount() << "  forward origin" << std::endl;

		// check origin //
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			// check pos //
			m.generalMotionPool().at(i).updMpm();
			m.generalMotionPool().at(i).getMpm(result);
			if (!s_is_equal(16, result, opo + i * 16, error[0])) 
			{
				std::cout << s.type() << "::kinPos() forward origin failed at " << i << std::endl;
				dsp(4, 4, result);
				dsp(4, 4, opo + i * 16);
			}
			// check vel //
			m.generalMotionPool().at(i).updMvs();
			m.generalMotionPool().at(i).getMva(result);
			if (!s_is_equal(6, result, ovo + i * 6, error[1]))
			{
				std::cout << s.type() << "::kinVel() forward origin failed at " << i << std::endl;
				dsp(1, 6, result);
				dsp(1, 6, ovo + i * 6);
			}
			// check acc //
			m.generalMotionPool().at(i).updMas();
			m.generalMotionPool().at(i).getMaa(result);
			if (!s_is_equal(6, result, oao + i * 6, error[2]))
			{
				std::cout << s.type() << "::dynAccAndFce() forward forward origin failed at " << i << ": acc not correct" << std::endl;
				dsp(1, 6, result);
				dsp(1, 6, oao + i * 6);
			}
		}
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)result[i] = m.motionPool().at(i).mf();
		if (!s_is_equal(m.motionPool().size(), result, ifo, error[3]))
		{
			std::cout << s.type() << "::dynAccAndFce() forward forward origin failed: fce not correct" << std::endl;
			dsp(1, m.motionPool().size(), result);
			dsp(1, m.motionPool().size(), ifo);
		}

		// set input //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).setMp(ipt[i]);
			m.motionPool().at(i).setMv(ivt[i]);
			m.motionPool().at(i).setMa(iat[i]);
		}
		// compute //
		m.init();
		s.kinPos();
		m.init();
		s.kinVel();
		m.init();
		s.dynAccAndFce();
		m.init();
		if (dynamic_cast<aris::dynamic::UniversalSolver *>(&s))dynamic_cast<aris::dynamic::UniversalSolver *>(&s)->cptGeneralJacobi();
		m.init();
		if (dynamic_cast<aris::dynamic::UniversalSolver *>(&s))dynamic_cast<aris::dynamic::UniversalSolver *>(&s)->cptGeneralInverseDynamicMatrix();
		std::cout << "iter count:" << s.iterCount() << "  forward" << std::endl;

		// check //
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			m.generalMotionPool().at(i).updMpm();
			m.generalMotionPool().at(i).getMpm(result);
			if (!s_is_equal(16, result, opt + 16 * i, error[0]))
			{
				std::cout << s.type() << "::kinPos() forward failed at " << i << std::endl;
				dsp(4, 4, result);
				dsp(4, 4, opt + 16 * i);
			}
			m.generalMotionPool().at(i).updMvs();
			m.generalMotionPool().at(i).getMva(result);
			if (!s_is_equal(6, result, ovt + 6 * i, error[1]))
			{
				std::cout << s.type() << "::kinVel() forward failed at " << i << std::endl;
				dsp(1, 6, result);
				dsp(1, 6, opt + 6 * i);
			}
			m.generalMotionPool().at(i).updMas();
			m.generalMotionPool().at(i).getMaa(result);
			if (!s_is_equal(6, result, oat + 6 * i, error[2]))
			{
				std::cout << s.type() << "::dynAccAndFce() forward forward failed at " << i << ": acc not correct" << std::endl;
				dsp(1, 6, result);
				dsp(1, 6, opt + 6 * i);
			}

		}
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)result[i] = m.motionPool().at(i).mf();
		if (!s_is_equal(m.motionPool().size(), result, ift, error[3]))
		{
			std::cout << s.type() << "::dynAccAndFce() forward forward failed: fce not correct" << std::endl;
			dsp(1, m.motionPool().size(), result);
			dsp(1, m.motionPool().size(), ift);
		}
		if (dynamic_cast<aris::dynamic::UniversalSolver *>(&s))
		{
			auto u = dynamic_cast<aris::dynamic::UniversalSolver *>(&s);

			aris::Size m = u->model()->partPool().size() * 6;
			aris::Size n = u->model()->motionPool().size() + u->model()->generalMotionPool().size() * 6;

			std::vector<double> part_vs(m, 0.0), part_as(m, 0.0);
			std::vector<double> mot_input(n, 0.0);
			for (aris::Size i = 0; i < u->model()->motionPool().size(); ++i)
			{
				mot_input.data()[i] = u->model()->motionPool().at(i).mv();
			}
			for (aris::Size i = 0; i < u->model()->generalMotionPool().size(); ++i)
			{
				s_vc(6, u->model()->generalMotionPool().at(i).mvs(), mot_input.data() + u->model()->motionPool().size() + 6 * i);
			}

			s_mm(m, 1, n, u->Jg(), mot_input.data(), part_vs.data());

			for (aris::Size i = 0; i < u->model()->partPool().size(); ++i)
			{
				if (!s_is_equal(6, u->model()->partPool().at(i).vs(), part_vs.data() + 6 * i, error[2]))
				{
					std::cout << s.type() << "::cptGeneralJacobi() forward failed" << std::endl;

					std::cout << "part " << u->model()->partPool().at(i).name() << ": " << i << " id:" << u->model()->partPool().at(i).id() << std::endl;
					dsp(1, 6, u->model()->partPool().at(i).vs());
					dsp(1, 6, part_vs.data() + 6 * i);

					break;
				}
			}

			// check cg //
			for (aris::Size i = 0; i < u->model()->motionPool().size(); ++i)
			{
				mot_input.data()[i] = u->model()->motionPool().at(i).ma();
			}
			for (aris::Size i = 0; i < u->model()->generalMotionPool().size(); ++i)
			{
				s_vc(6, u->model()->generalMotionPool().at(i).mas(), mot_input.data() + u->model()->motionPool().size() + 6 * i);
			}

			s_mm(m, 1, n, u->Jg(), mot_input.data(), part_as.data());
			s_va(m, u->cg(), part_as.data());

			for (aris::Size i = 0; i < u->model()->partPool().size(); ++i)
			{
				if (!s_is_equal(6, u->model()->partPool().at(i).as(), part_as.data() + 6 * i, error[2]))
				{
					std::cout << s.type() << "::cptGeneralJacobi() forward failed, because cg is not correct" << std::endl;

					std::cout << "part " << u->model()->partPool().at(i).name() << ": " << i << " id:" << u->model()->partPool().at(i).id() << std::endl;
					dsp(1, 6, u->model()->partPool().at(i).as());
					dsp(1, 6, part_as.data() + 6 * i);

					break;
				}
			}
		}
		if (dynamic_cast<aris::dynamic::UniversalSolver *>(&s))
		{
			auto u = dynamic_cast<aris::dynamic::UniversalSolver *>(&s);

			aris::Size m = u->model()->partPool().size() * 6;
			aris::Size n = u->model()->motionPool().size() + u->model()->generalMotionPool().size() * 6;

			std::vector<double> mf(n, 0.0), ma(n, 0.0), mf_compare(n, 0.0);

			for (aris::Size i = 0; i < u->model()->motionPool().size(); ++i)
			{
				ma.data()[i] = u->model()->motionPool().at(i).ma();
				mf_compare.data()[i] = u->model()->motionPool().at(i).mf();
			}
			for (aris::Size i = 0; i < u->model()->generalMotionPool().size(); ++i)
			{
				s_vc(6, u->model()->generalMotionPool().at(i).mas(), ma.data() + u->model()->motionPool().size() + 6 * i);
				std::fill(mf_compare.data() + u->model()->motionPool().size() + 6 * i, mf_compare.data() + u->model()->motionPool().size() + 6 * i + 6, 0.0);
			}

			s_mm(n, 1, n, u->M(), ma.data(), mf.data());
			s_va(n, u->h(), mf.data());

			if (!s_is_equal(n, mf.data(), mf_compare.data(), error[2])) 
			{
				std::cout << s.type() << "::cptGeneralInverseDynamicMatrix() forward failed" << std::endl;
				dsp(1, n, mf_compare.data());
				dsp(1, n, mf.data());
			}
		}



		// compute forward dynamic //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).activate(false);
			m.forcePool().at(i).activate(true);
			dynamic_cast<SingleComponentForce&>(m.forcePool().at(i)).setFce(ift[i]);
		}
		std::vector<double> before_jnt_cf, after_jnt_cf;
		for (auto &j : m.jointPool())for (int i = 0; i < j.dim(); ++i)before_jnt_cf.push_back(j.cf()[i]);
		m.init();
		s.dynAccAndFce();
		for (auto &j : m.jointPool())for (int i = 0; i < j.dim(); ++i)after_jnt_cf.push_back(j.cf()[i]);
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			m.generalMotionPool().at(i).updMas();
			m.generalMotionPool().at(i).getMaa(result);
			if (!s_is_equal(6, result, oat + 6 * i, error[2])) 
			{
				std::cout << s.type() << "::dynAccAndFce() forward forward failed at " << i << ": with force" << std::endl;
				dsp(1, 6, result);
				dsp(1, 6, oat + 6 * i);
			}
		}
		if (!s_is_equal(before_jnt_cf.size(), before_jnt_cf.data(), after_jnt_cf.data(), error[2]))
		{
			std::cout << s.type() << "::dynAccAndFce() forward forward failed when forward dynamic force" << std::endl;
		}


		/////////////////////////////////////////////////////反向，从输出到输入///////////////////////////////////////////////////
		for (auto &fce : m.forcePool())fce.activate(false);
		for (auto &mot : m.motionPool())mot.activate(false);
		for (auto &gm : m.generalMotionPool())gm.activate(true);
		// set ee origin status //
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			m.generalMotionPool().at(i).setMpm(opo + i*16);
			m.generalMotionPool().at(i).setMva(ovo + i*6);
			m.generalMotionPool().at(i).setMaa(oao + i*6);
		}

		// compute origin //
		m.init();
		s.kinPos();
		m.init();
		s.kinVel();
		m.init();
		s.dynAccAndFce();
		std::cout << "iter count:" << s.iterCount() << "  inverse origin" << std::endl;

		// check origin //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).updMp();
			result[i] = m.motionPool().at(i).mp();
		}
		if (!s_is_equal(m.motionPool().size(), result, ipo, error[4])) 
		{
			std::cout << s.type() << "::kinPos() inverse origin failed" << std::endl;
			dsp(1, m.motionPool().size(), result);
			dsp(1, m.motionPool().size(), ipo);
		}
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).updMv();
			result[i] = m.motionPool().at(i).mv();
		}
		if (!s_is_equal(m.motionPool().size(), result, ivo, error[5])) 
		{
			std::cout << s.type() << "::kinVel() inverse origin failed" << std::endl;
			dsp(1, m.motionPool().size(), result);
			dsp(1, m.motionPool().size(), ivo);
		}
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).updMa();
			result[i] = m.motionPool().at(i).ma();
		}
		if (!s_is_equal(m.motionPool().size(), result, iao, error[6]))
		{
			std::cout << s.type() << "::kinAcc() inverse origin failed" << std::endl;
			dsp(1, m.motionPool().size(), result);
			dsp(1, m.motionPool().size(), iao);
		}
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i) 
		{
			if (!s_is_equal(6, m.generalMotionPool().at(i).mfs(), ofo + 6 * i, error[7])) 
			{
				std::cout << s.type() << "::dynFce() inverse origin failed at " << i << std::endl;
				dsp(1, 6, m.generalMotionPool().at(i).mfs());
				dsp(1, 6, ofo + 6 * i);
			}
		}
		
		// set ee //
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			m.generalMotionPool().at(i).setMpm(opt + i * 16);
			m.generalMotionPool().at(i).setMva(ovt + i * 6);
			m.generalMotionPool().at(i).setMaa(oat + i * 6);
		}
		// compute //
		m.init();
		s.kinPos();
		m.init();
		s.kinVel();
		m.init();
		s.dynAccAndFce();
		m.init();
		if (dynamic_cast<aris::dynamic::UniversalSolver *>(&s))dynamic_cast<aris::dynamic::UniversalSolver *>(&s)->cptGeneralJacobi();
		m.init();
		if (dynamic_cast<aris::dynamic::UniversalSolver *>(&s))dynamic_cast<aris::dynamic::UniversalSolver *>(&s)->cptGeneralInverseDynamicMatrix();
		std::cout << "iter count:" << s.iterCount() << "  inverse" << std::endl;

		// check //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).updMp();
			result[i] = m.motionPool().at(i).mp();
		}
		if (!s_is_equal(m.motionPool().size(), result, ipt, error[4]))
		{
			std::cout << s.type() << "::kinPos() inverse failed" << std::endl;
			dsp(1, m.motionPool().size(), result);
			dsp(1, m.motionPool().size(), ipt);
		}
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).updMv();
			result[i] = m.motionPool().at(i).mv();
		}
		if (!s_is_equal(m.motionPool().size(), result, ivt, error[5]))
		{
			std::cout << s.type() << "::kinVel() inverse failed" << std::endl;
			dsp(1, m.motionPool().size(), result);
			dsp(1, m.motionPool().size(), ivt);
		}
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).updMa();
			result[i] = m.motionPool().at(i).ma();
		}
		if (!s_is_equal(m.motionPool().size(), result, iat, error[6]))
		{
			std::cout << s.type() << "::kinAcc() inverse failed" << std::endl;
			dsp(1, m.motionPool().size(), result);
			dsp(1, m.motionPool().size(), iat);
		}
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			if (!s_is_equal(6, m.generalMotionPool().at(i).mfs(), oft + 6 * i, error[7]))
			{
				std::cout << s.type() << "::dynFce() inverse failed at " << i << std::endl;
				dsp(1, 6, m.generalMotionPool().at(i).mfs());
				dsp(1, 6, oft + 6 * i);
			}
		}
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)if (!s_is_equal(6, m.generalMotionPool().at(i).mfs(), oft + 6 * i, error[7]))std::cout << s.type() << "::dynFce() inverse failed" << std::endl;
		if (dynamic_cast<aris::dynamic::UniversalSolver *>(&s))
		{
			// check Jg //
			auto u = dynamic_cast<aris::dynamic::UniversalSolver *>(&s);

			aris::Size m = u->model()->partPool().size() * 6;
			aris::Size n = u->model()->motionPool().size() + u->model()->generalMotionPool().size() * 6;

			std::vector<double> part_vs(m, 0.0), part_as(m, 0.0);
			std::vector<double> mot_input(n, 0.0);
			for (aris::Size i = 0; i < u->model()->motionPool().size(); ++i)
			{
				mot_input.data()[i] = u->model()->motionPool().at(i).mv();
			}
			for (aris::Size i = 0; i < u->model()->generalMotionPool().size(); ++i)
			{
				s_vc(6, u->model()->generalMotionPool().at(i).mvs(), mot_input.data() + u->model()->motionPool().size() + 6 * i);
			}

			s_mm(m, 1, n, u->Jg(), mot_input.data(), part_vs.data());

			for (aris::Size i = 0; i < u->model()->partPool().size(); ++i)
			{
				if (!s_is_equal(6, u->model()->partPool().at(i).vs(), part_vs.data() + 6 * i, error[2]))
				{
					std::cout << s.type() << "::cptGeneralJacobi() inverse failed" << std::endl;

					std::cout << "part " << u->model()->partPool().at(i).name() << ": " << i << " id:" << u->model()->partPool().at(i).id() << std::endl;
					dsp(1, 6, u->model()->partPool().at(i).vs());
					dsp(1, 6, part_vs.data() + 6 * i);

					break;
				}
			}

			// check cg //
			for (aris::Size i = 0; i < u->model()->motionPool().size(); ++i)
			{
				mot_input.data()[i] = u->model()->motionPool().at(i).ma();
			}
			for (aris::Size i = 0; i < u->model()->generalMotionPool().size(); ++i)
			{
				s_vc(6, u->model()->generalMotionPool().at(i).mas(), mot_input.data() + u->model()->motionPool().size() + 6 * i);
			}

			s_mm(m, 1, n, u->Jg(), mot_input.data(), part_as.data());
			s_va(m, u->cg(), part_as.data());

			for (aris::Size i = 0; i < u->model()->partPool().size(); ++i)
			{
				if (!s_is_equal(6, u->model()->partPool().at(i).as(), part_as.data() + 6 * i, error[2]))
				{
					std::cout << s.type() << "::cptGeneralJacobi() inverse failed, because cg is not correct" << std::endl;

					std::cout << "part " << u->model()->partPool().at(i).name() << ": " << i << " id:" << u->model()->partPool().at(i).id() << std::endl;
					dsp(1, 6, u->model()->partPool().at(i).as());
					dsp(1, 6, part_as.data() + 6 * i);

					break;
				}
			}
		}
		if (dynamic_cast<aris::dynamic::UniversalSolver *>(&s))
		{
			auto u = dynamic_cast<aris::dynamic::UniversalSolver *>(&s);

			aris::Size m = u->model()->partPool().size() * 6;
			aris::Size n = u->model()->motionPool().size() + u->model()->generalMotionPool().size() * 6;

			std::vector<double> mf(n, 0.0), ma(n, 0.0), mf_compare(n, 0.0);

			for (aris::Size i = 0; i < u->model()->motionPool().size(); ++i)
			{
				ma.data()[i] = u->model()->motionPool().at(i).ma();
				mf_compare.data()[i] = 0.0;
			}
			for (aris::Size i = 0; i < u->model()->generalMotionPool().size(); ++i)
			{
				s_vc(6, u->model()->generalMotionPool().at(i).mas(), ma.data() + u->model()->motionPool().size() + 6 * i);
				s_vc(6, u->model()->generalMotionPool().at(i).mfs(), mf_compare.data() + u->model()->motionPool().size() + 6 * i);
			}

			s_mm(n, 1, n, u->M(), ma.data(), mf.data());
			s_va(n, u->h(), mf.data());

			if (!s_is_equal(n, mf.data(), mf_compare.data(), error[2]))
			{
				std::cout << s.type() << "::cptGeneralInverseDynamicMatrix() forward failed" << std::endl;
				dsp(1, n, mf_compare.data());
				dsp(1, n, mf.data());
			}
			
		}
	}
}

void bench_solver(Model &m, aris::Size i, aris::Size bench_count, const double *ipo, const double *ivo, const double *iao, const double *ifo,
	const double *opo, const double *ovo, const double *oao, const double *ofo,
	const double *ipt, const double *ivt, const double *iat, const double *ift,
	const double *opt, const double *ovt, const double *oat, const double *oft, const double *error)
{
	double result1[18];
	auto &s = m.solverPool().at(i);

	// forward, input to ee //
	for (auto &mot : m.motionPool())mot.activate(true);
	for (auto &fce : m.forcePool())fce.activate(false);
	for (auto &gm : m.generalMotionPool())gm.activate(false);
	m.init();

	// init //
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
	/*
	// pos //
	std::cout << s.type() << "::forward computational pos time:" << aris::core::benchmark(bench_count, [&]()
	{
		if (count % 2)for (aris::Size i{ 0 }; i < m.motionPool().size(); ++i) m.motionPool().at(i).setMp(ipt[i]);
		else for (aris::Size i{ 0 }; i < m.motionPool().size(); ++i) m.motionPool().at(i).setMp(ipo[i]);

		s.kinPos();
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			m.generalMotionPool().at(i).updMpm();
			m.generalMotionPool().at(i).getMpm(result1);

			if (count < 2 && count % 2 && !s_is_equal(16, result1, opt + i * 16, error[0]))
				throw std::runtime_error(s.type() + "::kinPos() forward bench failed");
			if (count < 2 && (count + 1) % 2 && !s_is_equal(16, result1, opo + i*16, error[0]))
				throw std::runtime_error(s.type() + "::kinPos() forward bench origin pos failed");
		}
		++count;
	}) << std::endl;

	// vel //
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
			
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			m.generalMotionPool().at(i).updMvs();
			m.generalMotionPool().at(i).getMva(result1);
			if (!s_is_equal(6, result1, ovt + i*6, error[1]))std::cout << s.type() << "::kinVel() forward bench vel failed" << std::endl;
		}
	}) << std::endl;

	// dyn //
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
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			m.generalMotionPool().at(i).updMas();
			m.generalMotionPool().at(i).getMaa(result1);
			if (!s_is_equal(6, result1, oat + i*6, error[2]))	std::cout << s.type() << "::kinAcc() forward bench acc failed" << std::endl;
		}

	}) << std::endl;
	*/
	// dyn mat //
	for (aris::Size i = 0; i < m.motionPool().size(); ++i)
	{
		m.motionPool().at(i).setMp(ipt[i]);
		m.motionPool().at(i).setMv(ivt[i]);
		m.motionPool().at(i).setMa(iat[i]);
	}
	s.kinPos();
	s.kinVel();
	count = 0;
	std::cout << s.type() << "::forward computational dyn mat time:" << aris::core::benchmark(bench_count, [&]()
	{
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).setMp(ipt[i]);
			m.motionPool().at(i).setMv(ivt[i]);
			m.motionPool().at(i).setMa(iat[i]);
		}
		dynamic_cast<aris::dynamic::UniversalSolver&>(s).cptGeneralInverseDynamicMatrix();
	}) << std::endl;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (auto &mot : m.motionPool())mot.activate(false);
	for (auto &fce : m.forcePool())fce.activate(false);
	for (auto &gm : m.generalMotionPool())gm.activate(true);
	m.init();
	m.generalMotionPool().at(0).setMpm(opo);
	m.generalMotionPool().at(0).setMva(ovo);
	m.generalMotionPool().at(0).setMva(oao);
	s.kinPos();
	s.kinVel();
	s.dynAccAndFce();;
	/*
	// pos //
	count =  0;
	std::cout << s.type() << "::inverse computational pos time:" << aris::core::benchmark(bench_count, [&]()
	{
		if (count % 2)for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)m.generalMotionPool().at(i).setMpm(opt + 16 * i);
		else for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)m.generalMotionPool().at(i).setMpm(opo + 16 * i);

		// compute //
		s.kinPos();
		for (aris::Size i = 0; i < m.motionPool().size(); ++i) { m.motionPool().at(i).updMp(); result1[i] = m.motionPool().at(i).mp(); }

		if (count < 2 && count % 2 && !s_is_equal(m.motionPool().size(), result1, ipt, error[4]))
			std::cout << s.type() << "::kinPos() forward bench failed" << std::endl;
		if (count < 2 && (count + 1) % 2 && !s_is_equal(m.motionPool().size(), result1, ipo, error[4]))
			std::cout << s.type() << "::kinPos() forward bench origin failed" << std::endl;

		++count;
	}) << std::endl;

	// vel // 
	m.generalMotionPool().at(0).setMpm(opt);
	s.kinPos();
	count = 0;
	std::cout << s.type() << "::inverse computational vel time:" << aris::core::benchmark(bench_count, [&]()
	{
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)m.generalMotionPool().at(i).setMva(ovt + i * 6);
		s.kinVel();
		for (aris::Size i = 0; i < m.motionPool().size(); ++i) { m.motionPool().at(i).updMv(); result1[i] = m.motionPool().at(i).mv(); }
		if (!s_is_equal(m.motionPool().size(), result1, ivt, error[5])) std::cout << s.type() << "::kinVel() inverse bench vel failed" << std::endl;
	}) << std::endl;

	// dyn //
	m.generalMotionPool().at(0).setMpm(opt);
	m.generalMotionPool().at(0).setMva(ovt);
	s.kinPos();
	s.kinVel();
	count = 0;
	std::cout << s.type() << "::inverse computational acc time:" << aris::core::benchmark(bench_count, [&]()
	{
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)m.generalMotionPool().at(i).setMaa(oat + i * 6);
		s.dynAccAndFce();
		for (aris::Size i = 0; i < m.motionPool().size(); ++i) { m.motionPool().at(i).updMa(); result1[i] = m.motionPool().at(i).ma(); }
		if (!s_is_equal(m.motionPool().size(), result1, iat, error[6]))	std::cout << s.type() << "::kinAcc() inverse bench acc failed" << std::endl;
	}) << std::endl;
	*/
	// dyn mat //
	m.generalMotionPool().at(0).setMpm(opt);
	m.generalMotionPool().at(0).setMva(ovt);
	s.kinPos();
	s.kinVel();
	count = 0;
	std::cout << s.type() << "::inverse computational dyn mat time:" << aris::core::benchmark(bench_count, [&]()
	{
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)m.generalMotionPool().at(i).setMaa(oat + i * 6);
		dynamic_cast<aris::dynamic::UniversalSolver&>(s).cptGeneralInverseDynamicMatrix();
	}) << std::endl;
}

void test_single_body()
{
	std::cout << "test single body:" << std::endl;
	// 单刚体自由落体
	{
		
		aris::dynamic::Model m;
		auto &p = m.partPool().add<aris::dynamic::Part>();
		auto &s = m.solverPool().add<aris::dynamic::UniversalSolver>();

		m.init();

		p.setPe(std::array<double, 6>{0.1, 0.2, 0.3, 0.000423769269879415, 1.38980987554835, 1.79253453841257}.data(), "313");
		p.setVs(std::array<double, 6>{-0.244517963270725, 1.25737650310373, -0.874318412470487, -0.244517963270725, 1.25737650310373, -0.874318412470487}.data());
		p.setAs(std::array<double, 6>{0.0, -0.192390604845803, 0.136512424183815, 0.904633672502324, -1.24440604199266, 1.45568007018557}.data());

		s.kinPos();
		s.kinVel();
		s.dynAccAndFce();

		if (!s_is_equal(6, p.as(), std::array<double, 6>{0.2318970967746941, -9.2746063132688601, 0.6907262413433608, 0.0, 0.0, 0.0}.data(), 1e-10))std::cout << s.type() << "::dynAccAndFce() failed in single body" << std::endl;

	}
	// 两个重复的R副
	{
		std::cout << "test single body2:" << std::endl;

		// 测试两个完全一样的 R 副约束地面和杆件
		aris::dynamic::Model m;
		auto &p = m.partPool().add<aris::dynamic::Part>();

		double pm[16]{ 1,0,0,0.8 , 0,1,0,0.5 , 0,0,1,0.3 , 0,0,0,1 };
		auto &makI = p.markerPool().add<Marker>("mak_i", pm);
		auto &makJ = m.ground().markerPool().add<Marker>("mak_j");
		auto &r = m.jointPool().add<aris::dynamic::RevoluteJoint>("r", &makI, &makJ);
		auto &r2 = m.jointPool().add<aris::dynamic::RevoluteJoint>("r2", &makI, &makJ);
		auto &s = m.solverPool().add<aris::dynamic::UniversalSolver>();

		m.init();

		p.setPe(std::array<double, 6>{0.1, 0.2, 0.3, 0.000423769269879415, 1.38980987554835, 1.79253453841257}.data(), "313");
		p.setVs(std::array<double, 6>{-0.244517963270725, 1.25737650310373, -0.874318412470487, -0.244517963270725, 1.25737650310373, -0.874318412470487}.data());
		p.setAs(std::array<double, 6>{0.0, -0.192390604845803, 0.136512424183815, 0.904633672502324, -1.24440604199266, 1.45568007018557}.data());

		s.kinPos();
		s.kinVel();

		// 只有z轴能转，所以前5项应该都为0
		double pe[6], ve[6];
		makI.getVe(ve, pe, "123");
		if (s_norm(5, ve) > 1e-10 || s_norm(5, pe) > 1e-10)std::cout << __FILE__ << __LINE__ << ":kinPos() failed" << std::endl;

		p.setPe(std::array<double, 6>{0.67981395325965, -0.65410472319845, -0.29999999994657, -0.00000000000000, 0.00000000000000, 1.81686625023439}.data(), "123");
		p.setVs(std::array<double, 6>{-0.00000000002595, -0.00000000000266, -0.00000000000000, 0.00000000000000, -0.00000000000000, -0.87431841247392}.data());

		s.dynAccAndFce();
		if (!s_is_equal(6, p.as(), std::array<double, 6>{-0.0, -0.0, -0.0, 0.0, 0.0, -3.52496123913987}.data(), 1e-9))std::cout << __FILE__ << __LINE__ << ":dynAccAndFce() failed" << std::endl;

	}
}
void test_float_5_bar()
{
	std::cout << "test float 5 bar:" << std::endl;

	const double joint1_position[3]{ 0.7 , 0.8 , 0.0 };
	const double joint1_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint2_position[3]{ 1.0 , 0.8 , 0.0 };
	const double joint2_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint3_position[3]{ 1.1 , 1.1 , 0.0 };
	const double joint3_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint4_position[3]{ 0.75 , 1.2 , 0.0 };
	const double joint4_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint5_position[3]{ 0.6 , 1.0 , 0.0 };
	const double joint5_axis[3]{ 0.0 , 0.0 , 1.0 };

	aris::dynamic::Model m;
	auto &p1 = m.partPool().add<aris::dynamic::Part>("p1", std::array<double, 10>{2.0, 0.7, 0.8, 0.1, 4.0, 7.0, 4.8, 0.01, 0.03, 0.02}.data());
	auto &p2 = m.partPool().add<aris::dynamic::Part>("p2", std::array<double, 10>{1.8, 1.1, 0.6, 0.2, 5.0, 6.1, 3.8, 0.1, 0.03, 0.02}.data());
	auto &p3 = m.partPool().add<aris::dynamic::Part>("p3", std::array<double, 10>{0.5, 0.5, -0.4, 1.2, 6.1, 5.2, 2.8, 0.01, 0.3, 0.02}.data());
	auto &p4 = m.partPool().add<aris::dynamic::Part>("p4", std::array<double, 10>{1.6, 0.9, 0.3, -0.3, 4.0, 4.3, 3.2, 0.01, 0.03, 0.2}.data());
	auto &p5 = m.partPool().add<aris::dynamic::Part>("p5", std::array<double, 10>{1.7, 2.1, 1.1, 0.8, 7.0, 3.4, 5.8, 0.01, 0.03, 0.02}.data());
	auto &j1 = m.addRevoluteJoint(p2, p1, joint1_position, joint1_axis);
	auto &j2 = m.addRevoluteJoint(p3, p2, joint2_position, joint2_axis);
	auto &j3 = m.addRevoluteJoint(p4, p3, joint3_position, joint3_axis);
	auto &j4 = m.addRevoluteJoint(p5, p4, joint4_position, joint4_axis);
	auto &j5 = m.addRevoluteJoint(p1, p5, joint5_position, joint5_axis);
	auto &m1 = m.addMotion(j1);
	auto &s = m.solverPool().add<aris::dynamic::UniversalSolver>();
	
	auto &adams = m.simulatorPool().add<AdamsSimulator>();

	m.init();

	m1.setMp(0.0);
	m1.setMv(0.1);
	m1.setMa(0.2);

	p1.setVs(std::array<double, 6>{0.00000000000000, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.00000000000000}.data());
	p2.setVs(std::array<double, 6>{0.08000000000000, -0.07000000000000, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.10000000000000}.data());
	p3.setVs(std::array<double, 6>{0.01714285714286, 0.00857142857143, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.02142857142857}.data());
	p4.setVs(std::array<double, 6>{0.06428571428571, -0.03857142857143, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.06428571428571}.data());
	p5.setVs(std::array<double, 6>{0.06428571428571, -0.03857142857143, 0.00000000000000, 0.00000000000000, 0.00000000000000, 0.06428571428571}.data());

	//s.kinPos();
	//s.kinVel();
	s.dynAccAndFce();

	//s.kinPos();
	//s.dynAccAndFce();
	//dsp(4, 4, *p1.pm());
	//dsp(4, 4, *p2.pm());
	//dsp(4, 4, *p3.pm());
	//dsp(4, 4, *p4.pm());
	//dsp(4, 4, *p5.pm());


	//dsp(1, 6, p1.vs());
	//dsp(1, 6, p2.vs());
	//dsp(1, 6, p3.vs());
	//dsp(1, 6, p4.vs());
	//dsp(1, 6, p5.vs());

	//double data[6];
	//p1.getAa(data);
	//dsp(1, 6, data);
	//p2.getAa(data);
	//dsp(1, 6, data);
	//p3.getAa(data);
	//dsp(1, 6, data);
	//p4.getAa(data);
	//dsp(1, 6, data);
	//p5.getAa(data);
	//dsp(1, 6, data);


	if (!s_is_equal(6, p1.vs(), std::array<double, 6>{0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p2.vs(), std::array<double, 6>{0.08000000000000,   -0.07000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.10000000000000}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p3.vs(), std::array<double, 6>{0.01714285714286,   0.00857142857143,  0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02142857142857}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p4.vs(), std::array<double, 6>{0.06428571428571,   -0.03857142857143,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.06428571428571}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p5.vs(), std::array<double, 6>{0.06428571428571,   -0.03857142857143,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.06428571428571}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;

	if (!s_is_equal(6, p1.as(), std::array<double, 6>{-0.08859089478486, - 9.75558073147311, - 0.00018296503713,   0.00166526381296,   0.00049171676773, - 0.09405578038763}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p2.as(), std::array<double, 6>{0.07140910521514, - 9.89558073147311, - 0.00018296503713,   0.00166526381296,   0.00049171676773,   0.10594421961237}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p3.as(), std::array<double, 6>{-0.05136868452535, - 9.74505492286892, - 0.00018296503713,   0.00166526381296,   0.00049171676773, - 0.04458158899181}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p4.as(), std::array<double, 6>{0.03368654217674, - 9.82845708834653, - 0.00018296503713,   0.00166526381296,   0.00049171676773,   0.03148902526426}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
	if (!s_is_equal(6, p5.as(), std::array<double, 6>{0.05329075431848, - 9.84070972093511, - 0.00018296503713,   0.00166526381296,   0.00049171676773,   0.04782586871571}.data(), 1e-10))std::cout << __FILE__ << __LINE__ << ":failed" << std::endl;
}
void test_servo_press()
{
	std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

	// 设置重力 //
	const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
	model->environment().setGravity(gravity);

	// 添加变量 //
	model->calculator().addVariable("PI", "Number", aris::core::Matrix(aris::PI));

	// add part //
	auto &p1 = model->partPool().add<Part>("L1");

	// add joint //
	const double j1_pos[3]{ 2.800000e-001, 8.750000e-001, -2.500000e-001 };
	const double j1_axis[6]{ 0.0, 1.0, 0.0 };

	auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);

	// add actuation //
	auto &m1 = model->addMotion(j1);

	// add ee general motion //
	double pq_ee_i[]{ 2.800000e-001, 8.750000e-001, -2.500000e-001, 0, 0, 0, 1 };
	double pm_ee_i[16];
	double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	s_pq2pm(pq_ee_i, pm_ee_i);

	auto &makI = p1.markerPool().add<Marker>("ee_makI", pm_ee_i);
	auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
	auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ);

	// add solver
	auto &universal_solver = model->solverPool().add<UniversalSolver>();
	m1.activate(false);
	
	model->init();

	auto &m = *model;

	ee.setMpq(std::array<double, 7>{0, 0.2, 0, 0, 0, 0, 1}.data());
	ee.setMvs(std::array<double, 6>{0, 0.2, 0, 0, 0, 0}.data());
	//ee.setMas(std::array<double, 6>{0, 0, 0.7, 0, 0, 0}.data());
	universal_solver.kinPos();
}
void test_3R()
{
	// over constraint system
	// 本示例展示3轴SCARA机器人的建模过程，aris可以求解任何机构（串联、并联、混联、过约束、欠约束等）的正逆运动学、正逆动力学等问题
	// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
	// inertia_vector的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
	const double link1_position_and_euler321[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
	const double link1_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };
	const double link2_position_and_euler321[6]{ 1.0 , 0.0 , 0.0 , aris::PI / 2 , 0.0 , 0.0 };
	const double link2_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };
	const double link3_position_and_euler321[6]{ 1.0 , 1.0 , 0.0 , aris::PI , 0.0 , 0.0 };
	const double link3_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };

	// 定义关节的位置，以及轴线，SCARA包含3个转动副，转动轴线是Z轴
	const double joint1_position[3]{ 0.0 , 0.0 , 0.0 };
	const double joint1_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint2_position[3]{ 1.0 , 0.0 , 0.0 };
	const double joint2_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint3_position[3]{ 1.0 , 1.0 , 0.0 };
	const double joint3_axis[3]{ 0.0 , 0.0 , 1.0 };

	// 定义末端位置与321欧拉角，这个位置为机构起始时的位置
	const double end_effector_position_and_euler321[6]{ 0.0 , 1.0 , 0.0 , aris::PI , 0.0 , 0.0 };


	////////////////////////////////////////////////// 开始建模 ///////////////////////////////////////////////
	Model m;

	// 添加杆件，这里pe的意思为position and euler angle，函数的参数指定了位姿以及惯性向量
	auto &link1 = m.addPartByPe(link1_position_and_euler321, "321", link1_inertia_vector);
	auto &link2 = m.addPartByPe(link2_position_and_euler321, "321", link2_inertia_vector);
	auto &link3 = m.addPartByPe(link3_position_and_euler321, "321", link3_inertia_vector);

	// 添加关节，添加转动关节，前两个参数为关节连接的杆件，后两个参数定义了关节的位置与轴线
	auto &joint1 = m.addRevoluteJoint(link1, m.ground(), joint1_position, joint1_axis);
	auto &joint2 = m.addRevoluteJoint(link2, link1, joint2_position, joint2_axis);
	auto &joint3 = m.addRevoluteJoint(link3, link2, joint3_position, joint3_axis);

	// 添加驱动，驱动位于关节上
	auto &motion1 = m.addMotion(joint1);
	auto &motion2 = m.addMotion(joint2);
	auto &motion3 = m.addMotion(joint3);

	// 添加末端，第一个参数表明末端位于link3上，第二个参数表明末端的位姿是相对于地面的，后两个参数定义了末端的起始位姿
	auto &end_effector = m.addGeneralMotionByPe(link3, m.ground(), end_effector_position_and_euler321, "321");
	////////////////////////////////////////////////// 建模完毕 ///////////////////////////////////////////////


	auto &force1 = m.forcePool().add<SingleComponentForce>("f1", &motion1.makI(), &motion1.makJ(), 5);
	auto &force2 = m.forcePool().add<SingleComponentForce>("f2", &motion2.makI(), &motion2.makJ(), 5);
	auto &force3 = m.forcePool().add<SingleComponentForce>("f3", &motion3.makI(), &motion3.makJ(), 5);

	link1.geometryPool().add<ParasolidGeometry>("parasolid_geometry", "C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t");
	link2.geometryPool().add<ParasolidGeometry>("parasolid_geometry", "C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t");
	link3.geometryPool().add<ParasolidGeometry>("parasolid_geometry", "C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t");

	auto &solver = m.solverPool().add<UniversalSolver>();
	solver.setMaxError(1e-15);

	const double input_origin_p[3]{ 0.0 , 0.0 , 0.0 };
	const double input_origin_v[3]{ 0.0 , 0.0 , 0.0 };
	const double input_origin_a[3]{ 0.0 , 0.0 , 0.0 };
	const double input_origin_mf[3]{ 39.2, 0.0, 0.0 };
	const double output_origin_pm[16]{ -1,0,0,0,
		0, -1.0 , 0.0, 1.0,
		0, 0.0 , 1.0, 0.0,
		0,0,0,1 };
	const double output_origin_va[6]{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0 };
	const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0 };
	const double output_origin_mfs[6]{ 0.0 , -39.2 , 0.0, 0.0 , 0.0 , 39.2 };

	const double input_p[3]{ -0.0648537067263432, - 0.4611742608347527,0.5260279675610960 };
	const double input_v[3]{ 0.2647720948695498, - 0.5918279267633222,   0.6270558318937725 };
	const double input_a[3]{ 0.8080984807847047, - 0.7798913328042270,   0.1717928520195222 };
	const double input_mf[3]{ 63.2889513681872273, 13.0244588302560018, 1.9999999999999964 };
	const double output_pm[16]{ -1.0 , 0.0 , 0.0 , 0.5 ,
		0.0 , -1.0 , 0.0, 0.8,
		0.0 , 0.0 , 1.0 , 0.0,
		0.0 , 0.0 , 0.0 , 1.0 };
	const double output_va[6]{ 0.3 , -0.2 , 0.0 , 0.0 , 0.0 , 0.3 };
	const double output_aa[6]{ -0.0056868358720751,0.5326011894967951,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.2000000000000000 };
	const double output_mfs[6]{ -15.8974283255729407, - 49.3379293524304217,0.0000000000000000,0.0000000000000000,0.0000000000000000,51.3379293524304217 };

	const double error[8]{ 1e-9, 1e-9, 1e-8, 1e-8, 1e-9, 1e-9, 1e-8, 1e-8 };

	std::cout << "test 3R robot:" << std::endl;
	test_solver(m, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
		output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
		input_p, input_v, input_a, input_mf,
		output_pm, output_va, output_aa, output_mfs, error);
}
void test_ur5()
{
	try
	{
		const double error = 1e-10;

		const double input_origin_p[6]{ 0.0 , 0.0 , 0.2 , 0.3 , 0.1 , 0.0 };
		const double input_origin_v[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_a[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[6]{ 0.0 , - 58.7146084377919, - 15.27196793779189, 0.08355927033182553, - 2.208978793858849e-16, 4.440892098500626e-16 };
		const double output_origin_pm[16]{ -0.87319830445628,   0.47942553860421,   0.08761206554320,   0.77126396092354,
			0.09983341664683,   0.00000000000000,   0.99500416527803,   0.19103884280238,
			0.47703040785185,   0.87758256189037, - 0.04786268954661, - 0.07577133383697,
			0.00000000000000,   0.00000000000000,   0.00000000000000,   1.00000000000000 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ 331.41022567419924, - 57.57941566970891,   111.65453524745845, - 305.96717877205936, - 27.27506157298690,   0.00000000000009 };

		const double input_p[6]{ -0.2 , -0.3 , 0.5 , 0.4 , 0.1 , 0.2 };
		const double input_v[6]{ 0.93426722257942, -0.024823760537999, -0.89419018046124,   0.245922301638701, -1.23100367003297, -0.48185561218356 };
		const double input_a[6]{ 0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337,   0.131806583011732, -1.65802060177352 };
		const double input_mf[6]{ 3.597125027933981, - 59.48962385005701, - 15.75853251590728, 0.05996237522379046, - 0.2198225545568449,- 0.01337878841492246 };
		const double output_pm[16]{ -0.65941998050411,   0.69831352105816,  0.27843045023725,   0.76691425043133,
			0.23350446430995, - 0.16179239408332,   0.95880075425717,   0.03946313726684,
			0.71459145982812,   0.69726712780982, - 0.05637018730295,   0.05406976046047,
			0.00000000000000,   0.00000000000000,   0.00000000000000,   1.00000000000000 };
		const double output_va[6]{ 0.00532701661719,   0.71826926345644,   0.38888755008115,   0.41333575398384, - 1.25976861786569,   1.97742068465392 };
		const double output_aa[6]{ -1.00962853878322,   0.52234724380448,   0.38786009482355,   0.17228242874921, - 2.42549515902665,   0.80062985749427 };
		const double output_mfs[6]{ 147.06728488236050, - 47.85997209760284,   74.93903147977326, - 138.27415845097042,   15.35176588136645, - 0.01337878841486 };

		const double error2[8]{ 1e-10, 1e-10, 1e-10, 1e-9, 1e-10, 1e-10, 1e-9, 1e-9 };

		std::cout << "test ur5 robot:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_ur5);
		m.init();

		test_solver(m, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error2);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_stewart()
{
	try
	{	
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
		const double output_mfs[6]{ -1752.8168759636657796,-200.8968525620247192, 86.3334906336755807,   -816.6714933354393224, 1685.6093614991480081, 661.2063243054601571 };

		const double error[8]{ 1e-10, 1e-9, 1e-8, 1e-8, 1e-10, 1e-9, 1e-8, 1e-8 };

		std::cout << "test stewart robot:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_stewart);
		m.init();

		test_solver(m, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_ur5_on_stewart()
{
	try
	{
		const double input_origin_p[12]{ 0.0 , 0.0 , 0.2 , 0.3 , 0.1 , 0.0
			,2.0 , 2.0 , 2.0 , 2.0 , 2.0 , 2.0 };
		const double input_origin_v[12]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_a[12]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[12]{ -0.00131007958832, - 58.71460842317624, - 15.27196793399029,   0.08355927031103,   0.00000000000000,   0.00000000000000
			,   38.79115991258558,   90.14001822992438,   102.29642215058338, - 46.58560558137449,   12.29206464937079,   114.33889336911456 };
		const double output_origin_pm[16 * 2]{ -0.87319830445628,   0.47942553860421,   0.08761206554320,   0.77126396092354,
			0.09983341664683,   0.00000000000000,   0.99500416527803,   0.19103884280238,
			0.47703040785185,   0.87758256189037, -0.04786268954661, -0.07577133383697,
			0.00000000000000,   0.00000000000000,   0.00000000000000,   1.00000000000000
			,1,0,0,0,0, 0.999999999751072,2.2312668404904e-05,1.7078344386197,0, -2.23126684049141e-05,0.999999999751072,0.577658198650165,0,0,0,1 };
		const double output_origin_va[12]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[12]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[12]{ 331.41005441296738, - 57.57941565538285,   111.65282914193104, - 305.96717869591038, - 27.27504747818914,   0.00000000000034
			,27.19113760205170,   265.80040464945978,   30.92404622013744,   18.34356297165392, - 0.66137519668830,   48.40251269235812 };

		const double input_p[12]{ -0.2 , -0.3 , 0.5 , 0.4 , 0.1 , 0.2
			,2.15,2.03,1.98,1.68,2.22,2.01 };
		const double input_v[12]{ 0.93426722257942, -0.024823760537999, -0.89419018046124,   0.245922301638701, -1.23100367003297, -0.48185561218356
			,0.687,1.521,-0.325,0.665,1.225,-0.999 };
		const double input_a[12]{ 0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337,   0.131806583011732, -1.65802060177352
			,1.687,0.521,-1.325,1.665,0.225,-1.999 };
		const double input_mf[12]{ 66.24194156738221, - 1.05905897103295,   7.17155195304943,   0.08817930939522, - 2.78563372294673, - 0.03526665204329
			,   2320.18419686476227,   289.98437373182901,   2815.25099394435256, - 4095.97671710069153,   3990.45076484181300, - 5074.18245692169967 };
		const double output_pm[16*2]{ -0.65941998050411,   0.69831352105816,  0.27843045023725,   0.76691425043133,
			0.23350446430995, -0.16179239408332,   0.95880075425717,   0.03946313726684,
			0.71459145982812,   0.69726712780982, -0.05637018730295,   0.05406976046047,
			0.00000000000000,   0.00000000000000,   0.00000000000000,   1.00000000000000
			,0.654617242227831, -0.16813527373803,0.737025641279234,0.0674004103296998,
			0.286892301165042,0.957269694021347, -0.0364354283699648,1.66351811346172,
			-0.699406229390514,0.235298241883176,0.674881962758251,0.907546391448817,
			0,0,0,1 };
		const double output_va[12]{ 0.00532701661719,   0.71826926345644,   0.38888755008115,   0.41333575398384, -1.25976861786569,   1.97742068465392
			,-1.67602445813444,0.322144550146041,1.43386389933679, -4.13258637478856,0.229701802785213,2.06026880988191 };
		const double output_aa[12]{ -1.00962853878322,   0.52234724380448,   0.38786009482355,   0.17228242874921, -2.42549515902665,   0.80062985749427
			,-3.99625983193204, -4.52459258496676,3.82662285536541, -4.70386456087171,10.2271223856012,12.7760010719168 };
		const double output_mfs[12]{ 61.71570587230033, - 58.66601320855561,   111.79172160413337, - 59.39130412572584,   8.82357386446111, - 0.03526665204313
			,-1896.46477427475475, - 193.52698104653490, - 11.32950583387003, - 832.90636926666343,   1751.85130306653605,   677.71751107782461 };

		const double error[8]{ 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9 };

		std::cout << "test ur on stewart:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_ur5_on_stewart);

		test_solver(m, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_multi_systems()
{
	try
	{
		const double input_origin_p[15]{ 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,2.0 , 2.0 , 2.0 , 2.0 , 2.0 , 2.0 };
		const double input_origin_v[15]{ 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_a[15]{ 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[15]{ 39.2, 0.0, 0.0
			,34.8372347443935, 0.179465241344625, 26.1146353297101, 0.0, 0.0, 0.0
			,11.3871054640557503, 13.6991667810515523, -5.9717817946918421, 5.5825903452181880, 50.4497942853426551, 41.0950030749182460 };
		const double output_origin_pm[16*3]{ -1.0 ,0.0 ,0.0 ,0.0 ,0.0 , -1.0 , 0.0, 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1
			,1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0
			,1,0,0,0,0, 0.999999999751072,2.2312668404904e-05,1.7078344386197,0, -2.23126684049141e-05,0.999999999751072,0.577658198650165,0,0,0,1 };
		const double output_origin_va[18]{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[18]{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[18]{ 0.0 , -39.2 , 0.0, 0.0 , 0.0 , 39.2
			,1.53575836143848,38.0002468240561,16.8933419758238,8.4241178308505,-16.6881665634178,51.5626922658449
			,27.1911376020517253, 99.2601846909161623, 30.9203302634324899, 5.1114448379402990, -0.6600651170996999, -10.3120957308179584 };

		const double input_p[15]{ -0.0648537067263432, -0.4611742608347527,0.5260279675610960
			,-0.084321840829742,0.111235847475406,0.163501201249858,0.41316722587035, -0.0861578092597486,0.229246197281016
			,2.15,2.03,1.98,1.68,2.22,2.01 };
		const double input_v[15]{ 0.2647720948695498, -0.5918279267633222,   0.6270558318937725
			,0.93426722257942, -0.024823760537999, -0.89419018046124,   0.245922301638701, -1.23100367003297, -0.48185561218356
			,0.687,1.521,-0.325,0.665,1.225,-0.999 };
		const double input_a[15]{ 0.8080984807847047, -0.7798913328042270,   0.1717928520195222
			,0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337,   0.131806583011732, -1.65802060177352
			,1.687,0.521,-1.325,1.665,0.225,-1.999 };
		const double input_mf[15]{ 63.2889513681872273, 13.0244588302560018, 1.9999999999999964
			,24.6359418510515, -3.06678992657553, 13.4565070365958,   -15.0336821069307,   -0.786112551012351,   -1.93281931696021
			,2124.4812403533132965, 275.7606350348426645, 2757.4743715794970740,   -3994.5016219944400291, 3892.2226684026377370, -4810.0920919994805445 };
		const double output_pm[48]{ -1.0 , 0.0 , 0.0 , 0.5 ,
			0.0 , -1.0 , 0.0, 0.8,
			0.0 , 0.0 , 1.0 , 0.0,
			0.0 , 0.0 , 0.0 , 1.0
			,0.863013488544127, -0.284074444773496,   0.417743256579356, -0.137731283515364,
			0.387677110267304,   0.902605554641921, -0.187108714132569, -0.343275971674581,
			-0.323904579723239,   0.323426842664891,   0.889089928341408, -0.0474940394315194,
			0,   0,   0,   1
			,0.654617242227831, -0.16813527373803,0.737025641279234,0.0674004103296998,
			0.286892301165042,0.957269694021347, -0.0364354283699648,1.66351811346172,
			-0.699406229390514,0.235298241883176,0.674881962758251,0.907546391448817,
			0,0,0,1	};
		const double output_va[18]{ 0.3 , -0.2 , 0.0 , 0.0 , 0.0 , 0.3 
			,-1.93242030056314,   0.500930573127293,   0.577926916892486, -0.399682310201935, -0.66053331463003, -0.857440373970742
			,-1.67602445813444,0.322144550146041,1.43386389933679, -4.13258637478856,0.229701802785213,2.06026880988191 };
		const double output_aa[18]{ -0.0056868358720751,0.5326011894967951,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.2000000000000000
			,1.07075600145293,   0.349116022890415,   2.0925775293411, -1.77982973680254, -0.927893632540704,   0.0659817357654945
			,-3.99625983193204, -4.52459258496676,3.82662285536541, -4.70386456087171,10.2271223856012,12.7760010719168 };
		const double output_mfs[18]{ -15.8974283255729407, -49.3379293524304217,0.0000000000000000,0.0000000000000000,0.0000000000000000,51.3379293524304217
			,8.44990411304192, 54.7768126462764, 23.2058019399381, 18.6214939645874,   -51.751313528282, 82.047228392192
			,-1752.8168759636657796,-200.8968525620247192, 86.3334906336755807,   -816.6714933354393224, 1685.6093614991480081, 661.2063243054601571 };

		const double error[8]{ 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9 };

		std::cout << "test multi systems:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_multi);

		test_solver(m, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void bench_3R()
{
	try
	{
		// over constraint system
		// 本示例展示3轴SCARA机器人的建模过程，aris可以求解任何机构（串联、并联、混联、过约束、欠约束等）的正逆运动学、正逆动力学等问题
		// 定义3个杆件的位置与321欧拉角，以及10维的惯量向量
		// inertia_vector的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
		const double link1_position_and_euler321[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double link1_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };
		const double link2_position_and_euler321[6]{ 1.0 , 0.0 , 0.0 , aris::PI / 2 , 0.0 , 0.0 };
		const double link2_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };
		const double link3_position_and_euler321[6]{ 1.0 , 1.0 , 0.0 , aris::PI , 0.0 , 0.0 };
		const double link3_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };

		// 定义关节的位置，以及轴线，SCARA包含3个转动副，转动轴线是Z轴
		const double joint1_position[3]{ 0.0 , 0.0 , 0.0 };
		const double joint1_axis[3]{ 0.0 , 0.0 , 1.0 };
		const double joint2_position[3]{ 1.0 , 0.0 , 0.0 };
		const double joint2_axis[3]{ 0.0 , 0.0 , 1.0 };
		const double joint3_position[3]{ 1.0 , 1.0 , 0.0 };
		const double joint3_axis[3]{ 0.0 , 0.0 , 1.0 };

		// 定义末端位置与321欧拉角，这个位置为机构起始时的位置
		const double end_effector_position_and_euler321[6]{ 0.0 , 1.0 , 0.0 , aris::PI , 0.0 , 0.0 };


		////////////////////////////////////////////////// 开始建模 ///////////////////////////////////////////////
		Model m;

		// 添加杆件，这里pe的意思为position and euler angle，函数的参数指定了位姿以及惯性向量
		auto &link1 = m.addPartByPe(link1_position_and_euler321, "321", link1_inertia_vector);
		auto &link2 = m.addPartByPe(link2_position_and_euler321, "321", link2_inertia_vector);
		auto &link3 = m.addPartByPe(link3_position_and_euler321, "321", link3_inertia_vector);

		// 添加关节，添加转动关节，前两个参数为关节连接的杆件，后两个参数定义了关节的位置与轴线
		auto &joint1 = m.addRevoluteJoint(link1, m.ground(), joint1_position, joint1_axis);
		auto &joint2 = m.addRevoluteJoint(link2, link1, joint2_position, joint2_axis);
		auto &joint3 = m.addRevoluteJoint(link3, link2, joint3_position, joint3_axis);

		// 添加驱动，驱动位于关节上
		auto &motion1 = m.addMotion(joint1);
		auto &motion2 = m.addMotion(joint2);
		auto &motion3 = m.addMotion(joint3);

		// 添加末端，第一个参数表明末端位于link3上，第二个参数表明末端的位姿是相对于地面的，后两个参数定义了末端的起始位姿
		auto &end_effector = m.addGeneralMotionByPe(link3, m.ground(), end_effector_position_and_euler321, "321");
		////////////////////////////////////////////////// 建模完毕 ///////////////////////////////////////////////


		auto &force1 = m.forcePool().add<SingleComponentForce>("f1", &motion1.makI(), &motion1.makJ(), 5);
		auto &force2 = m.forcePool().add<SingleComponentForce>("f2", &motion2.makI(), &motion2.makJ(), 5);
		auto &force3 = m.forcePool().add<SingleComponentForce>("f3", &motion3.makI(), &motion3.makJ(), 5);

		auto &solver = m.solverPool().add<UniversalSolver>();
		solver.setMaxError(1e-15);

		const double input_origin_p[3]{ 0.0 , 0.0 , 0.0 };
		const double input_origin_v[3]{ 0.0 , 0.0 , 0.0 };
		const double input_origin_a[3]{ 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[3]{ 39.2, 0.0, 0.0 };
		const double output_origin_pm[16]{ -1,0,0,0,
			0, -1.0 , 0.0, 1.0,
			0, 0.0 , 1.0, 0.0,
			0,0,0,1 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ 0.0 , -39.2 , 0.0, 0.0 , 0.0 , 39.2 };

		const double input_p[3]{ -0.0648537067263432, -0.4611742608347527,0.5260279675610960 };
		const double input_v[3]{ 0.2647720948695498, -0.5918279267633222,   0.6270558318937725 };
		const double input_a[3]{ 0.8080984807847047, -0.7798913328042270,   0.1717928520195222 };
		const double input_mf[3]{ 63.2889513681872273, 13.0244588302560018, 1.9999999999999964 };
		const double output_pm[16]{ -1.0 , 0.0 , 0.0 , 0.5 ,
			0.0 , -1.0 , 0.0, 0.8,
			0.0 , 0.0 , 1.0 , 0.0,
			0.0 , 0.0 , 0.0 , 1.0 };
		const double output_va[6]{ 0.3 , -0.2 , 0.0 , 0.0 , 0.0 , 0.3 };
		const double output_aa[6]{ -0.0056868358720751,0.5326011894967951,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.2000000000000000 };
		const double output_mfs[6]{ -15.8974283255729407, -49.3379293524304217,0.0000000000000000,0.0000000000000000,0.0000000000000000,51.3379293524304217 };

		const double error[8]{ 1e-9, 1e-9, 1e-8, 1e-8, 1e-9, 1e-9, 1e-8, 1e-8 };

		std::cout << "bench 3R robot:" << std::endl;

		bench_solver(m, 0, 10000, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void bench_ur5()
{
	try
	{
		const double input_origin_p[6]{ 0.0 , 0.0 , 0.2 , 0.3 , 0.1 , 0.0 };
		const double input_origin_v[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_a[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[6]{ 0.0 , -58.7146084377919, -15.27196793779189, 0.08355927033182553, -2.208978793858849e-16, 4.440892098500626e-16 };
		const double output_origin_pm[16]{ -0.87319830445628,   0.47942553860421,   0.08761206554320,   0.77126396092354,
			0.09983341664683,   0.00000000000000,   0.99500416527803,   0.19103884280238,
			0.47703040785185,   0.87758256189037, -0.04786268954661, -0.07577133383697,
			0.00000000000000,   0.00000000000000,   0.00000000000000,   1.00000000000000 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ 331.41022567419924, -57.57941566970891,   111.65453524745845, -305.96717877205936, -27.27506157298690,   0.00000000000009 };

		const double input_p[6]{ -0.2 , -0.3 , 0.5 , 0.4 , 0.1 , 0.2 };
		const double input_v[6]{ 0.93426722257942, -0.024823760537999, -0.89419018046124,   0.245922301638701, -1.23100367003297, -0.48185561218356 };
		const double input_a[6]{ 0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337,   0.131806583011732, -1.65802060177352 };
		const double input_mf[6]{ 3.597125027933981, -59.48962385005701, -15.75853251590728, 0.05996237522379046, -0.2198225545568449,-0.01337878841492246 };
		const double output_pm[16]{ -0.65941998050411,   0.69831352105816,  0.27843045023725,   0.76691425043133,
			0.23350446430995, -0.16179239408332,   0.95880075425717,   0.03946313726684,
			0.71459145982812,   0.69726712780982, -0.05637018730295,   0.05406976046047,
			0.00000000000000,   0.00000000000000,   0.00000000000000,   1.00000000000000 };
		const double output_va[6]{ 0.00532701661719,   0.71826926345644,   0.38888755008115,   0.41333575398384, -1.25976861786569,   1.97742068465392 };
		const double output_aa[6]{ -1.00962853878322,   0.52234724380448,   0.38786009482355,   0.17228242874921, -2.42549515902665,   0.80062985749427 };
		const double output_mfs[6]{ 147.06728488236050, -47.85997209760284,   74.93903147977326, -138.27415845097042,   15.35176588136645, -0.01337878841486 };

		const double error[8]{ 1e-10, 1e-10, 1e-10, 1e-9, 1e-8, 1e-8, 1e-8, 1e-8 };

		std::cout << "bench 6R robot:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_ur5);

		bench_solver(m, 0, 10000, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void bench_stewart()
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
		const double output_mfs[6]{ -1752.8168759636657796,-200.8968525620247192, 86.3334906336755807,   -816.6714933354393224, 1685.6093614991480081, 661.2063243054601571 };

		const double error2[8]{ 1e-10, 1e-9, 1e-8, 1e-8, 1e-10, 1e-9, 1e-8, 1e-8 };

		std::cout << "bench stewart robot:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_stewart);

		bench_solver(m, 0, 10000, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error2);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void bench_multi_systems()
{
	try
	{
		const double input_origin_p[15]{ 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,2.0 , 2.0 , 2.0 , 2.0 , 2.0 , 2.0 };
		const double input_origin_v[15]{ 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_a[15]{ 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[15]{ 39.2, 0.0, 0.0
			,34.8372347443935, 0.179465241344625, 26.1146353297101, 0.0, 0.0, 0.0
			,11.3871054640557503, 13.6991667810515523, -5.9717817946918421, 5.5825903452181880, 50.4497942853426551, 41.0950030749182460 };
		const double output_origin_pm[16 * 3]{ -1.0 ,0.0 ,0.0 ,0.0 ,0.0 , -1.0 , 0.0, 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1
			,1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0
			,1,0,0,0,0, 0.999999999751072,2.2312668404904e-05,1.7078344386197,0, -2.23126684049141e-05,0.999999999751072,0.577658198650165,0,0,0,1 };
		const double output_origin_va[18]{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[18]{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0
			,0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[18]{ 0.0 , -39.2 , 0.0, 0.0 , 0.0 , 39.2
			,1.53575836143848,38.0002468240561,16.8933419758238,8.4241178308505,-16.6881665634178,51.5626922658449
			,27.1911376020517253, 99.2601846909161623, 30.9203302634324899, 5.1114448379402990, -0.6600651170996999, -10.3120957308179584 };

		const double input_p[15]{ -0.0648537067263432, -0.4611742608347527,0.5260279675610960
			,-0.084321840829742,0.111235847475406,0.163501201249858,0.41316722587035, -0.0861578092597486,0.229246197281016
			,2.15,2.03,1.98,1.68,2.22,2.01 };
		const double input_v[15]{ 0.2647720948695498, -0.5918279267633222,   0.6270558318937725
			,0.93426722257942, -0.024823760537999, -0.89419018046124,   0.245922301638701, -1.23100367003297, -0.48185561218356
			,0.687,1.521,-0.325,0.665,1.225,-0.999 };
		const double input_a[15]{ 0.8080984807847047, -0.7798913328042270,   0.1717928520195222
			,0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337,   0.131806583011732, -1.65802060177352
			,1.687,0.521,-1.325,1.665,0.225,-1.999 };
		const double input_mf[15]{ 63.2889513681872273, 13.0244588302560018, 1.9999999999999964
			,24.6359418510515, -3.06678992657553, 13.4565070365958,   -15.0336821069307,   -0.786112551012351,   -1.93281931696021
			,2124.4812403533132965, 275.7606350348426645, 2757.4743715794970740,   -3994.5016219944400291, 3892.2226684026377370, -4810.0920919994805445 };
		const double output_pm[48]{ -1.0 , 0.0 , 0.0 , 0.5 ,
			0.0 , -1.0 , 0.0, 0.8,
			0.0 , 0.0 , 1.0 , 0.0,
			0.0 , 0.0 , 0.0 , 1.0
			,0.863013488544127, -0.284074444773496,   0.417743256579356, -0.137731283515364,
			0.387677110267304,   0.902605554641921, -0.187108714132569, -0.343275971674581,
			-0.323904579723239,   0.323426842664891,   0.889089928341408, -0.0474940394315194,
			0,   0,   0,   1
			,0.654617242227831, -0.16813527373803,0.737025641279234,0.0674004103296998,
			0.286892301165042,0.957269694021347, -0.0364354283699648,1.66351811346172,
			-0.699406229390514,0.235298241883176,0.674881962758251,0.907546391448817,
			0,0,0,1 };
		const double output_va[18]{ 0.3 , -0.2 , 0.0 , 0.0 , 0.0 , 0.3
			,-1.93242030056314,   0.500930573127293,   0.577926916892486, -0.399682310201935, -0.66053331463003, -0.857440373970742
			,-1.67602445813444,0.322144550146041,1.43386389933679, -4.13258637478856,0.229701802785213,2.06026880988191 };
		const double output_aa[18]{ -0.0056868358720751,0.5326011894967951,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.2000000000000000
			,1.07075600145293,   0.349116022890415,   2.0925775293411, -1.77982973680254, -0.927893632540704,   0.0659817357654945
			,-3.99625983193204, -4.52459258496676,3.82662285536541, -4.70386456087171,10.2271223856012,12.7760010719168 };
		const double output_mfs[18]{ -15.8974283255729407, -49.3379293524304217,0.0000000000000000,0.0000000000000000,0.0000000000000000,51.3379293524304217
			,8.44990411304192, 54.7768126462764, 23.2058019399381, 18.6214939645874,   -51.751313528282, 82.047228392192
			,-1752.8168759636657796,-200.8968525620247192, 86.3334906336755807,   -816.6714933354393224, 1685.6093614991480081, 661.2063243054601571 };

		const double error[8]{ 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9 };

		std::cout << "bench multi systems:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_multi);

		bench_solver(m, 0, 10000, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}

void test_ur5_calibration()
{
	const double PI = 3.14159265358979;
	
	Model rbt;
	
	// 设置重力 //
	const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
	rbt.environment().setGravity(gravity);

	// add parts //
	// iv : inertia vector : m cx cy cz Ixx Iyy Izz Ixy Ixz Iyz
	// pm : 4x4 pose matrix
	// pe : 6x1 position and euler angle
	double iv[10], pm[16];

	const double p1_pe[6]{ 0.0, 0.00193, 0.089159 - 0.02561, 0, 0, 0 };
	const double p1_iv[10]{ 3.7, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p1_pe, pm, "321");
	s_iv2iv(pm, p1_iv, iv);
	auto &p1 = rbt.partPool().add<Part>("L1", iv);

	const double p2_pe[6]{ 0.2125, -0.024201 + 0.13585, 0.089159, 0.0, 0.0, 0.0 };
	const double p2_iv[10]{ 8.393, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p2_pe, pm, "321");
	s_iv2iv(pm, p2_iv, iv);
	auto &p2 = rbt.partPool().add<Part>("L2", iv);

	const double p3_pe[6]{ 0.425 + 0.110949, 0.13585 - 0.1197, 0.089159 + 0.01634, 0, 0, 0 };
	const double p3_iv[10]{ 2.275, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p3_pe, pm, "321");
	s_iv2iv(pm, p3_iv, iv);
	auto &p3 = rbt.partPool().add<Part>("L3", iv);

	const double p4_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197, 0.089159, 0, 0, 0 };
	const double p4_iv[10]{ 1.219, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p4_pe, pm, "321");
	s_iv2iv(pm, p4_iv, iv);
	auto &p4 = rbt.partPool().add<Part>("L4", iv);

	const double p5_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159, 0, 0, 0 };
	const double p5_iv[10]{ 1.219, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p5_pe, pm, "321");
	s_iv2iv(pm, p5_iv, iv);
	auto &p5 = rbt.partPool().add<Part>("L5", iv);

	const double p6_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, 0, 0 };
	const double p6_iv[10]{ 0.1879, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
	s_pe2pm(p6_pe, pm, "321");
	s_iv2iv(pm, p6_iv, iv);
	auto &p6 = rbt.partPool().add<Part>("L6", iv);

	// add joints //
	const double j1_pos[6]{ 0.0, 0.0, 0.089159 }, j1_axis[6]{ 0.0, 0.0, 1.0 };
	const double j2_pos[6]{ 0.0, 0.13585, 0.089159 }, j2_axis[6]{ 0.0, 1.0, 0.0 };
	const double j3_pos[6]{ 0.425, 0.13585 - 0.1197, 0.089159 }, j3_axis[6]{ 0.0, 1.0, 0.0 };
	const double j4_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197, 0.089159 }, j4_axis[6]{ 0.0, 1.0, 0.0 };
	const double j5_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 }, j5_axis[6]{ 0.0, 0.0, 1.0 };
	const double j6_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465 }, j6_axis[6]{ 0.0, 1.0, 0.0 };

	auto &j1 = rbt.addRevoluteJoint(p1, rbt.ground(), j1_pos, j1_axis);
	auto &j2 = rbt.addRevoluteJoint(p2, p1, j2_pos, j2_axis);
	auto &j3 = rbt.addRevoluteJoint(p3, p2, j3_pos, j3_axis);
	auto &j4 = rbt.addRevoluteJoint(p4, p3, j4_pos, j4_axis);
	auto &j5 = rbt.addRevoluteJoint(p5, p4, j5_pos, j5_axis);
	auto &j6 = rbt.addRevoluteJoint(p6, p5, j6_pos, j6_axis);

	// add actuation //
	auto &m1 = rbt.addMotion(j1);
	auto &m2 = rbt.addMotion(j2);
	auto &m3 = rbt.addMotion(j3);
	auto &m4 = rbt.addMotion(j4);
	auto &m5 = rbt.addMotion(j5);
	auto &m6 = rbt.addMotion(j6);

	m1.setFrcCoe(std::array<double, 3>{0.5, 0.6, 7}.data());
	m2.setFrcCoe(std::array<double, 3>{0.2, 0.1, 8}.data());
	m3.setFrcCoe(std::array<double, 3>{0.3, 0.2, 9}.data());
	m4.setFrcCoe(std::array<double, 3>{0.1, 0.3, 10}.data());
	m5.setFrcCoe(std::array<double, 3>{0.5, 0.4, 11}.data());
	m6.setFrcCoe(std::array<double, 3>{0.3, 0.5, 12}.data());

	// add end effector //
	double ee_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093 + 0.0823, 0.089159 - 0.09465, PI, PI / 2, 0 };
	auto &ee = rbt.addGeneralMotionByPe(p6, rbt.ground(), ee_pe, "321");


	auto &fee = rbt.forcePool().add<aris::dynamic::GeneralForce>("fce", &ee.makI(), &ee.makJ());
	fee.setFce(std::array<double, 6>{0.1, 0.2, 0.3, 0.1, 0.2, 0.3, }.data());

	auto &f3 = rbt.forcePool().add<aris::dynamic::GeneralForce>("fce", &j3.makI(), &j3.makJ());
	f3.setFce(std::array<double, 6>{0.1, 0.2, 0.3, 0.1, 0.2, 0.3, }.data());

	// add solvers //
	auto &forward_kinematic = rbt.solverPool().add<ForwardKinematicSolver>();

	// add calibrator //
	auto &calibrator = rbt.calibratorPool().add<Calibrator>();


	// allocate memory //
	forward_kinematic.allocateMemory();
	ee.activate(false);
	calibrator.allocateMemory();


	double t = 0.1;
	double q[6]{ 0.1 + 0.1*sin(t),0.2 + 0.2*sin(t * 2),0.3 + 0.3*sin(t * 3),0.4 + 0.4*sin(t * 4),0.5 + 0.5*sin(t * 5),0.6 + 0.6*sin(t * 6) };
	double dq[6]{ 0.1*cos(t), 0.2 * 2 * cos(t * 2), 0.3 * 3 * cos(t * 3), 0.4 * 4 * cos(t * 4), 0.5 * 5 * cos(t * 5), 0.6 * 6 * cos(t * 6) };
	double ddq[6]{ -0.1*sin(t), -0.2 * 2 * 2 * sin(t * 2), -0.3 * 3 * 3 * sin(t * 3), -0.4 * 4 * 4 * sin(t * 4), -0.5 * 5 * 5 * sin(t * 5), -0.6 * 6 * 6 * sin(t * 6) };

	// 分别设置电机的位置、速度、加速度， mp的意思为motion position...
	for (aris::Size i = 0; i < 6; ++i)
	{
		rbt.motionPool().at(i).setMp(q[i]);
		rbt.motionPool().at(i).setMv(dq[i]);
		rbt.motionPool().at(i).setMa(ddq[i]);
	}

	forward_kinematic.kinPos();
	forward_kinematic.kinVel();
	forward_kinematic.dynAccAndFce();

	calibrator.clb();


	dsp(calibrator.m(), calibrator.n(), calibrator.A());
	dsp(calibrator.m(), 1, calibrator.b());
	dsp(calibrator.n(), 1, calibrator.x());

	dsp(calibrator.m(), 1, calibrator.b());

	double b[6];
	s_mm(calibrator.m(), 1, calibrator.n(), calibrator.A(), calibrator.x(), b);
	dsp(calibrator.m(), 1, b);


}

auto test_clb()->void
{
	std::cout << "-------------------------------------------------" << std::endl;

	// 创建机器人 //
	aris::dynamic::PumaParam param;
	param.d1 = 0.3295;
	param.a1 = 0.04;
	param.a2 = 0.275;
	param.d3 = 0.0;
	param.a3 = 0.025;
	param.d4 = 0.28;

	param.tool0_pe[2] = 0.078;
	auto m = aris::dynamic::createModelPuma(param);

	// 为已有机器人添加 辨识器，也可以在xml里定义
	auto &clb = m->calibratorPool().add<aris::dynamic::Calibrator>();
	//auto &clb = m->calibratorPool()[0];

	// 以下代码为辨识器分配内存
	for (auto &ee : m->generalMotionPool())ee.activate(false);
	clb.allocateMemory();

	// 以下为当前已知的电机参数 //
	double q[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
	double dq[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
	double ddq[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
	double f[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };

	// 设置当前电机的位置、速度、加速度
	for (int i = 0; i < m->motionPool().size(); ++i)
	{
		m->motionPool()[i].setMp(q[i]);
		m->motionPool()[i].setMv(dq[i]);
		m->motionPool()[i].setMa(ddq[i]);
	}

	// 求正解、速度正解、加速度解
	m->solverPool().at(1).kinPos();
	m->solverPool().at(1).kinVel();
	m->solverPool().at(1).dynAccAndFce();


	// 设置当前电机的力
	for (int i = 0; i < m->motionPool().size(); ++i)
	{
		m->motionPool()[i].setMf(f[i]);
	}

	// 开始辨识 //
	clb.clb();


	// 辨识会得到 A 、 x 、 b 这样的矩阵和向量
	// 理论上 A * x = b
	// A为 m * n 维，x 为 n * 1 维，b维 m * 1维
	// 
	// m 为电机个数，也就是当前点的方程数，比如这里就是6
	// n 为待辨识的参数，它为杆件数(不含地面) * 10 + 电机 * 3，因此这里是78
	//
	// 杆件的辨识参数如下，其中xyz为质心位置：
	// m m*x m*y m*z Ixx Iyy Izz Ixy Ixz Iyz
	// 电机的辨识参数如下，也就是静摩擦力、粘性摩擦系数、电机转子惯量：
	// fs kv ki
	// 其中电机摩擦力计算为： 
	// f = sig(v)*fs + kv * v + ki * a
	//
	// x为当前的惯量值，注意它并不是辨识出来的结果，它仅仅保存了当前model中各个杆件的惯量和电机参数
	// b为当前的电机出力
	// A为观测矩阵


	std::cout << "m:" << clb.m() << std::endl;
	std::cout << "n:" << clb.n() << std::endl;

	// 以下打印各个矩阵：
	dsp(clb.m(), clb.n(), clb.A());
	dsp(clb.m(), 1, clb.b());
	dsp(clb.n(), 1, clb.x());
}


void test_model_solver()
{
	std::cout << std::endl << "-----------------test model compute---------------------" << std::endl;
	test_single_body();
	test_float_5_bar();
	test_servo_press();
	test_3R();
	test_ur5();
	test_stewart();
	test_ur5_on_stewart();
	test_multi_systems();

	bench_3R();
	bench_ur5();
	bench_stewart();
	bench_multi_systems();

	//test_ur5_calibration();

	//test_clb();
	std::cout << "-----------------test model compute finished------------" << std::endl << std::endl;
}

