#include "test_dynamic_model_compute.h"
#include <iostream>
#include <aris_dynamic.h>

#include<type_traits>

using namespace aris::dynamic;

const char xml_file_6R[] =
"<model>"
"	<environment type=\"Environment\" gravity=\"{0 , -9.8 , 0 , 0 , 0 , 0}\"/>"
"	<variable_pool type=\"VariablePoolElement\"/>"
"	<part_pool type=\"PartPoolElement\">"
"		<ground type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R0j type=\"Marker\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\"/>"
"				<origin type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"			</marker_pool>"
"		</ground>"
"		<part0 type=\"Part\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R0i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R1j type=\"Marker\" active=\"true\" pe=\"{0.127581045131803 , -0.255551891814505 , 0.743785121868611 , 1.39801111882379 , 0.92874853281329 , 2.99910369947683}\"/>"
"			</marker_pool>"
"		</part0>"
"		<part1 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.83142459280181 , 1.5646920289751 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R1i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R2j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 6.21426444075139 , 1.09203109440023 , 1.87858662805362}\"/>"
"			</marker_pool>"
"		</part1>"
"		<part2 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322217}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R2i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R3j type=\"Marker\" active=\"true\" pe=\"{0 , -0.996734365258792 , 0.0807502638519182 , 1.5707963267949 , 7.66425882286152e-17 , 4.71238898038469}\"/>"
"			</marker_pool>"
"		</part2>"
"		<part3 type=\"Part\" active=\"true\" pe=\"{1.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R3i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R4j type=\"Marker\" active=\"true\" pe=\"{1.53280055883457 , 0.0630122434114775 , 0.777786541421676 , 4.71238898038469 , 1.11022302462516e-16 , 1.5707963267949}\"/>"
"			</marker_pool>"
"		</part3>"
"		<part4 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R4i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R5j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 0.299854573312955 , 1.27790273658208 , 3.56732345865295}\"/>"
"			</marker_pool>"
"		</part4>"
"		<part5 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 1.928105009803 , 1.5084212451233 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R5i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<end_effector type=\"Marker\" active=\"true\" pe=\"{1.6840663068739 , 0.615533362716626 , -2.33680109411025 , -0 , 1.5084212451233 , 1.2134876437868}\"/>"
"			</marker_pool>"
"		</part5>"
"	</part_pool>"
"	<joint_pool type=\"JointPoolElement\">"
"		<R0 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\"/>"
"		<R1 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\"/>"
"		<R2 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\"/>"
"		<R3 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\"/>"
"		<R4 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\"/>"
"		<R5 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\"/>"
"	</joint_pool>"
"	<motion_pool type=\"MotionPoolElement\">"
"		<M0 type=\"Motion\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M1 type=\"Motion\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M2 type=\"Motion\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M3 type=\"Motion\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M4 type=\"Motion\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M5 type=\"Motion\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"	</motion_pool>"
"	<general_motion_pool type=\"GeneralMotionPoolElement\">"
"		<ee_mot type=\"GeneralMotion\" active=\"true\" prt_m=\"part5\" prt_n=\"ground\" mak_i=\"end_effector\" mak_j=\"origin\"/>"
"	</general_motion_pool>"
"	<force_pool type=\"ForcePoolElement\">"
"		<F1 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" component=\"5\"/>"
"		<F2 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" component=\"5\"/>"
"		<F3 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" component=\"5\"/>"
"		<F4 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" component=\"5\"/>"
"		<F5 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" component=\"5\"/>"
"		<F6 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" component=\"5\"/>"
"	</force_pool>"
"	<solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"		<us type=\"UniversalSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"	</solver_pool>"
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
"        <m1 active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m2 active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m3 active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m4 active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m5 active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m6 active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"    </motion_pool>"
"	<force_pool type=\"ForcePoolElement\">"
"		<f1 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" component=\"2\"/>"
"		<f2 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" component=\"2\"/>"
"		<f3 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" component=\"2\"/>"
"		<f4 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" component=\"2\"/>"
"		<f5 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" component=\"2\"/>"
"		<f6 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" component=\"2\"/>"
"	</force_pool>"
"    <general_motion_pool type=\"GeneralMotionPoolElement\" default_child_type=\"GeneralMotion\">"
"        <ee_mot type=\"GeneralMotion\" active=\"false\" prt_m=\"up\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"    </general_motion_pool>"
"    <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"        <us type=\"UniversalSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"    </solver_pool>"
"</model>";

const char xml_file_multi[] =
"<model>"
"    <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"    <variable_pool type=\"VariablePoolElement\" default_child_type=\"Matrix\">"
"        <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"        <Mot_friction type=\"MatrixVariable\">{0, 0, 0}</Mot_friction>"
"    </variable_pool>"
"    <part_pool type=\"PartPoolElement\" default_child_type=\"Part\">"
"        <part type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\"/>"
"        <part_1 type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <joint_0_i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                <joint_1_j type=\"Marker\" active=\"true\" pe=\"{1 , 0 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\"/>"
"        </part_1>"
"        <part_2 type=\"Part\" active=\"true\" pe=\"{1 , 0 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <joint_1_i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 3.06161699786838e-17 , 0 , 3.06161699786838e-17}\"/>"
"                <joint_2_j type=\"Marker\" active=\"true\" pe=\"{1 , 6.12323399573677e-17 , 0 , 3.06161699786838e-17 , 0 , 3.06161699786838e-17}\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\"/>"
"        </part_2>"
"        <part_3 type=\"Part\" active=\"true\" pe=\"{1 , 1 , 0 , 1.5707963267949 , 0 , 1.5707963267949}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <joint_2_i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 5.49778714378214 , 0 , 5.49778714378214}\"/>"
"                <general_motion_0_i type=\"Marker\" active=\"true\" pe=\"{1 , 1.22464679914735e-16 , 0 , -0 , 0 , -0}\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\"/>"
"        </part_3>"
"		<part0 type=\"Part\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R0i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R1j type=\"Marker\" active=\"true\" pe=\"{0.127581045131803 , -0.255551891814505 , 0.743785121868611 , 1.39801111882379 , 0.92874853281329 , 2.99910369947683}\"/>"
"			</marker_pool>"
"		</part0>"
"		<part1 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.83142459280181 , 1.5646920289751 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R1i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R2j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 6.21426444075139 , 1.09203109440023 , 1.87858662805362}\"/>"
"			</marker_pool>"
"		</part1>"
"		<part2 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322217}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R2i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R3j type=\"Marker\" active=\"true\" pe=\"{0 , -0.996734365258792 , 0.0807502638519182 , 1.5707963267949 , 7.66425882286152e-17 , 4.71238898038469}\"/>"
"			</marker_pool>"
"		</part2>"
"		<part3 type=\"Part\" active=\"true\" pe=\"{1.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R3i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R4j type=\"Marker\" active=\"true\" pe=\"{1.53280055883457 , 0.0630122434114775 , 0.777786541421676 , 4.71238898038469 , 1.11022302462516e-16 , 1.5707963267949}\"/>"
"			</marker_pool>"
"		</part3>"
"		<part4 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R4i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<R5j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 0.299854573312955 , 1.27790273658208 , 3.56732345865295}\"/>"
"			</marker_pool>"
"		</part4>"
"		<part5 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 1.928105009803 , 1.5084212451233 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"			<marker_pool type=\"MarkerPoolElement\">"
"				<R5i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"				<end_effector type=\"Marker\" active=\"true\" pe=\"{1.6840663068739 , 0.615533362716626 , -2.33680109411025 , -0 , 1.5084212451233 , 1.2134876437868}\"/>"
"			</marker_pool>"
"		</part5>"
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
"				<R0j type=\"Marker\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\"/>"
"                <joint_0_j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                <general_motion_0_j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"            </marker_pool>"
"        </ground>"
"    </part_pool>"
"    <joint_pool type=\"JointPoolElement\">"
"        <joint_0 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part_1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <joint_1 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part_2\" prt_n=\"part_1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"        <joint_2 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part_3\" prt_n=\"part_2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" cf=\"{0 , 0 , 0 , 0 , 0}\"/>"
"		<R0 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\"/>"
"		<R1 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\"/>"
"		<R2 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\"/>"
"		<R3 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\"/>"
"		<R4 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\"/>"
"		<R5 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\"/>"
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
"        <motion_0 type=\"Motion\" active=\"true\" prt_m=\"part_1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <motion_1 type=\"Motion\" active=\"true\" prt_m=\"part_2\" prt_n=\"part_1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"        <motion_2 type=\"Motion\" active=\"true\" prt_m=\"part_3\" prt_n=\"part_2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" cf=\"{0}\" frc_coe=\"{0 , 0 , 0}\" component=\"5\" mp=\"0\" mv=\"0\" ma=\"0\"/>"
"		<M0 type=\"Motion\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M1 type=\"Motion\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M2 type=\"Motion\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M3 type=\"Motion\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M4 type=\"Motion\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"		<M5 type=\"Motion\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        <m1 active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m2 active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m3 active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m4 active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m5 active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m6 active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"    </motion_pool>"
"	<force_pool type=\"ForcePoolElement\">"
"        <F3R1 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part_1\" prt_n=\"ground\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" component=\"5\"/>"
"        <F3R2 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part_2\" prt_n=\"part_1\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" component=\"5\"/>"
"        <F3R3 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part_3\" prt_n=\"part_2\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" component=\"5\"/>"
"		<F1 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" component=\"5\"/>"
"		<F2 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" component=\"5\"/>"
"		<F3 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" component=\"5\"/>"
"		<F4 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" component=\"5\"/>"
"		<F5 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" component=\"5\"/>"
"		<F6 type=\"SingleComponentForce\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" component=\"5\"/>"
"		<f1 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" component=\"2\"/>"
"		<f2 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" component=\"2\"/>"
"		<f3 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" component=\"2\"/>"
"		<f4 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" component=\"2\"/>"
"		<f5 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" component=\"2\"/>"
"		<f6 type=\"SingleComponentForce\" active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" component=\"2\"/>"
"	</force_pool>"
"    <general_motion_pool type=\"GeneralMotionPoolElement\" default_child_type=\"GeneralMotion\">"
"        <general_motion_0 type=\"GeneralMotion\" active=\"true\" prt_m=\"part_3\" prt_n=\"ground\" mak_i=\"general_motion_0_i\" mak_j=\"general_motion_0_j\" cf=\"{0 , 0 , 0 , 0 , 0 , 0}\"/>"
"		<ee_mot type=\"GeneralMotion\" active=\"true\" prt_m=\"part5\" prt_n=\"ground\" mak_i=\"end_effector\" mak_j=\"origin\"/>"
"        <ee_mot type=\"GeneralMotion\" active=\"false\" prt_m=\"up\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"    </general_motion_pool>"
"    <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"        <us type=\"UniversalSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"    </solver_pool>"
"</model>";


void test_solver(Model &m, const double *ipo, const double *ivo, const double *iao, const double *ifo, 
	const double *opo, const double *ovo, const double *oao, const double *ofo, 
	const double *ipt, const double *ivt, const double *iat, const double *ift, 
	const double *opt, const double *ovt, const double *oat, const double *oft, const double *error)
{
	// 测试运动学正解、反解、动力学正向正解、反解/反向正解、反解
	for (auto &s : m.solverPool())
	{
		/////////////////////////////////////////////////////正向，从输入到输出///////////////////////////////////////////////////
		double result1[18], result2[18], result3[18], result4[18];
		
		// set topology //
		for (auto &fce : m.forcePool())fce.activate(false);
		for (auto &mot : m.motionPool())mot.activate(true);
		for (auto &gm : m.generalMotionPool())gm.activate(false);
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
		std::cout << "iter count:" << s.iterCount() << "  forward origin" << std::endl;

		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			// get result //
			m.generalMotionPool().at(i).updMpm();
			m.generalMotionPool().at(i).getMpm(result1);
			m.generalMotionPool().at(i).updMvs();
			m.generalMotionPool().at(i).getMva(result2);
			m.generalMotionPool().at(i).updMas();
			m.generalMotionPool().at(i).getMaa(result3);
			// check //
			if (!s_is_equal(16, result1, opo + i*16, error[0]))std::cout << s.type() << "::kinPos() forward origin failed" << std::endl;
			if (!s_is_equal(6, result2, ovo + i*6, error[1]))std::cout << s.type() << "::kinVel() forward origin failed" << std::endl;
			if (!s_is_equal(6, result3, oao + i*6, error[2]))std::cout << s.type() << "::dynAccAndFce() forward forward origin failed: acc not correct" << std::endl;
		}
		
		for (int i = 0; i < m.motionPool().size(); ++i)result4[i] = m.motionPool().at(i).mf();
		if (!s_is_equal(m.motionPool().size(), result4, ifo, error[3]))std::cout << s.type() << "::dynAccAndFce() forward forward origin failed: fce not correct" << std::endl;

		// set input //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).setMp(ipt[i]);
			m.motionPool().at(i).setMv(ivt[i]);
			m.motionPool().at(i).setMa(iat[i]);
		}
		// compute kinematic and inverse dynamic //
		s.init();
		s.kinPos();
		s.init();
		s.kinVel();
		s.init();
		s.dynAccAndFce();
		std::cout << "iter count:" << s.iterCount() << "  forward" << std::endl;

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
		if (!s_is_equal(6, result3, oat, error[2]))std::cout << s.type() << "::dynAccAndFce() forward forward failed: acc not correct" << std::endl;
		if (!s_is_equal(m.motionPool().size(), result4, ift, error[3]))std::cout << s.type() << "::dynAccAndFce() forward forward failed: fce not correct" << std::endl;
		// compute forward dynamic //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).activate(false);
			m.forcePool().at(i).activate(true);
			dynamic_cast<SingleComponentForce&>(m.forcePool().at(i)).setFce(ift[i]);
		}
		s.init();
		s.dynAccAndFce();
		m.generalMotionPool().at(0).updMas();
		m.generalMotionPool().at(0).getMaa(result3);
		if (!s_is_equal(6, result3, oat, error[2]))std::cout << s.type() << "::dynAccAndFce() forward forward failed: with force" << std::endl;

		/////////////////////////////////////////////////////反向，从输出到输入///////////////////////////////////////////////////
		// set topology //
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

		// compute //
		s.init();
		s.kinPos();
		s.init();
		s.kinVel();
		s.init();
		s.dynAccAndFce();
		std::cout << "iter count:" << s.iterCount() << "  inverse origin" << std::endl;

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
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)if (!s_is_equal(6, m.generalMotionPool().at(i).mfs(), ofo + 6 * i, error[7]))std::cout << s.type() << "::dynFce() inverse origin failed" << std::endl;
		

		// set ee status //
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)
		{
			m.generalMotionPool().at(i).setMpm(opt + i * 16);
			m.generalMotionPool().at(i).setMva(ovt + i * 6);
			m.generalMotionPool().at(i).setMaa(oat + i * 6);
		}
		// compute //
		s.init();
		s.kinPos();
		s.init();
		s.kinVel();
		s.init();
		s.dynAccAndFce();
		std::cout << "iter count:" << s.iterCount() << "  inverse" << std::endl;

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
		for (aris::Size i = 0; i < m.generalMotionPool().size(); ++i)if (!s_is_equal(6, m.generalMotionPool().at(i).mfs(), oft + 6 * i, error[7]))std::cout << s.type() << "::dynFce() inverse failed" << std::endl;
	}
}

void bench_solver(Model &m, aris::Size i, aris::Size bench_count, const double *ipo, const double *ivo, const double *iao, const double *ifo,
	const double *opo, const double *ovo, const double *oao, const double *ofo,
	const double *ipt, const double *ivt, const double *iat, const double *ift,
	const double *opt, const double *ovt, const double *oat, const double *oft, const double *error)
{
	double result1[18];
	auto &s = m.solverPool().at(i);

	for (auto &mot : m.motionPool())mot.activate(true);
	for (auto &fce : m.forcePool())fce.activate(false);
	for (auto &gm : m.generalMotionPool())gm.activate(false);
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






	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (auto &mot : m.motionPool())mot.activate(false);
	for (auto &fce : m.forcePool())fce.activate(false);
	for (auto &gm : m.generalMotionPool())gm.activate(true);
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
}

void test_single_body()
{
	std::cout << "test single body:" << std::endl;

	// under constraint system
	aris::dynamic::Model m;
	auto &p = m.partPool().add<aris::dynamic::Part>();
	auto &s = m.solverPool().add<aris::dynamic::UniversalSolver>();

	s.allocateMemory();

	p.setPe(std::array<double, 6>{0.1, 0.2, 0.3, 0.000423769269879415, 1.38980987554835, 1.79253453841257}.data(), "313");
	p.setVs(std::array<double, 6>{-0.244517963270725, 1.25737650310373, -0.874318412470487, -0.244517963270725, 1.25737650310373, -0.874318412470487}.data());
	p.setAs(std::array<double, 6>{0.0, -0.192390604845803, 0.136512424183815, 0.904633672502324, -1.24440604199266, 1.45568007018557}.data());

	s.kinPos();
	s.kinVel();
	s.dynAccAndFce();

	if (!s_is_equal(6, p.as(), std::array<double, 6>{0.2318970967746941, -9.2746063132688601, 0.6907262413433608, 0.0, 0.0, 0.0}.data(), 1e-10))std::cout << s.type() << "::dynAccAndFce() failed in single body" << std::endl;
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
void test_6R()
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

		const double error2[8]{ 1e-10, 1e-10, 1e-10, 1e-9, 1e-10, 1e-10, 1e-9, 1e-9 };

		std::cout << "test 6R robot:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_6R);

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

		//bench_solver(m, 0, 10000, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
		//	output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
		//	input_p, input_v, input_a, input_mf,
		//	output_pm, output_va, output_aa, output_mfs, error);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void bench_6R()
{
	try
	{
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

		const double error[8]{ 1e-10, 1e-10, 1e-10, 1e-9, 1e-10, 1e-10, 1e-9, 1e-9 };

		std::cout << "bench 6R robot:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_6R);

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

void test_model_compute()
{
	std::cout << std::endl << "-----------------test model compute---------------------" << std::endl;
	test_single_body();
	test_3R();
	test_6R();
	test_stewart();
	test_multi_systems();

	bench_3R();
	bench_6R();
	bench_stewart();
	bench_multi_systems();

	test_ur5_calibration();
	std::cout << "-----------------test model compute finished------------" << std::endl << std::endl;
}

