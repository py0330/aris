#include "test_dynamic_model_multi.h"
#include <iostream>
#include <aris_dynamic.h>

#include<type_traits>

using namespace aris::dynamic;
/*
"        <fake_ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <origin pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r4i pe=\"{ 1.5,1,0,0,0,0 }\"/>"
"            </marker_pool>"
"        </fake_ground>"
*/
const char xml_file_under_constraint[] =
"<model>"
"    <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"    <variable_pool type=\"VariablePoolElement\" default_child_type=\"Matrix\">"
"        <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"        <Mot_friction type=\"MatrixVariable\">{0, 0, 0}</Mot_friction>"
"    </variable_pool>"
"    <part_pool type=\"PartPoolElement\" default_child_type=\"Part\">"
"        <fake_ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <origin pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r5i pe=\"{ 2.0,1.5,0,0,0,0 }\"/>"
"            </marker_pool>"
"        </fake_ground>"
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
"        <part4 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1.5,1.5,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <r4i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r5j pe=\"{ 0.5,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part3.x_t\"/>"
"            </geometry_pool>"
"        </part4>"
"        <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <origin pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r4i pe=\"{ 1.5,1,0,0,0,0 }\"/>"
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
"    </part_pool>"
"    <joint_pool type=\"JointPoolElement\">"
"        <r1 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part1\" prt_n=\"fake_ground\" mak_i=\"r1i\" mak_j=\"r1j\"/>"
"        <r2 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\"/>"
"        <r3 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"        <r4 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"r4i\" mak_j=\"r4j\"/>"
"        <r5 active=\"true\" type=\"RevoluteJoint\" prt_m=\"fake_ground\" prt_n=\"part4\" mak_i=\"r5i\" mak_j=\"r5j\"/>"
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
"        <mr active=\"true\" slave_id=\"1\" prt_m=\"part1\" prt_n=\"fake_ground\" mak_i=\"r1i\" mak_j=\"r1j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
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
"        <gs type=\"LltGroundDividedSolver\"/>"
"        <ps type=\"LltPartDividedSolver\"/>"
"        <ds type=\"UniversalSolver\"/>"
"    </solver_pool>"
"</model>";






void test_solver_multi()
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

		ds.plotRelation();

		for (auto &mot : m.motionPool()) 
		{
			mot.setMp(2.0);
			mot.setMv(0.0);
			mot.setMa(0.0);
		}
		m.motionPool().at(0).setMp(0.0);
		m.motionPool().at(0).setMa(1.0);

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
		
		for (auto &mot : m.motionPool())std::cout << mot.mf() << std::endl;

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

void test_model_multi()
{
	std::cout << std::endl << "-----------------test model---------------------" << std::endl;
	test_solver_multi();
	std::cout << "-----------------test model finished------------" << std::endl << std::endl;
}

