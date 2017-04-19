#include "test_dynamic_model_6R.h"
#include <iostream>
#include <aris.h>

using namespace aris::dynamic;

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<Root>"
"    <model>"
"        <environment type=\"Environment\" gravity=\"{0 , -9.8 , 0 , 0 , 0 , 0}\"/>"
"        <script type=\"ScriptPoolObject\"/>"
"        <variable_pool type=\"VariablePoolObject\"/>"
"        <akima_pool type=\"AkimaPoolObject\"/>"
"        <part_pool type=\"PartPoolObject\">"
"            <ground type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolObject\">"
"                    <R0j type=\"Marker\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\"/>"
"                    <origin type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                </marker_pool>"
"            </ground>"
"            <part0 type=\"Part\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolObject\">"
"                    <R0i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R1j type=\"Marker\" active=\"true\" pe=\"{0.127581045131803 , -0.255551891814505 , 0.743785121868611 , 1.39801111882379 , 0.92874853281329 , 2.99910369947683}\"/>"
"                </marker_pool>"
"            </part0>"
"            <part1 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.83142459280181 , 1.5646920289751 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolObject\">"
"                    <R1i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R2j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 6.21426444075139 , 1.09203109440023 , 1.87858662805362}\"/>"
"                </marker_pool>"
"            </part1>"
"            <part2 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322217}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolObject\">"
"                    <R2i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R3j type=\"Marker\" active=\"true\" pe=\"{0 , -0.996734365258792 , 0.0807502638519182 , 1.5707963267949 , 7.66425882286152e-17 , 4.71238898038469}\"/>"
"                </marker_pool>"
"            </part2>"
"            <part3 type=\"Part\" active=\"true\" pe=\"{1.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolObject\">"
"                    <R3i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R4j type=\"Marker\" active=\"true\" pe=\"{1.53280055883457 , 0.0630122434114775 , 0.777786541421676 , 4.71238898038469 , 1.11022302462516e-16 , 1.5707963267949}\"/>"
"                </marker_pool>"
"            </part3>"
"            <part4 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolObject\">"
"                    <R4i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <R5j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 0.299854573312955 , 1.27790273658208 , 3.56732345865295}\"/>"
"                </marker_pool>"
"            </part4>"
"            <part5 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 1.928105009803 , 1.5084212451233 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolObject\">"
"                    <R5i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                    <end_effector type=\"Marker\" active=\"true\" pe=\"{1.6840663068739 , 0.615533362716626 , -2.33680109411025 , -0 , 1.5084212451233 , 1.2134876437868}\"/>"
"                </marker_pool>"
"            </part5>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolObject\">"
"            <R0 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\"/>"
"            <R1 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\"/>"
"            <R2 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\"/>"
"            <R3 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\"/>"
"            <R4 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\"/>"
"            <R5 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolObject\">"
"            <M0 type=\"Motion\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M1 type=\"Motion\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M2 type=\"Motion\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M3 type=\"Motion\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M4 type=\"Motion\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <M5 type=\"Motion\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolObject\">"
"            <ee_mot type=\"GeneralMotion\" active=\"true\" prt_m=\"part5\" prt_n=\"ground\" mak_i=\"end_effector\" mak_j=\"origin\"/>"
"        </general_motion_pool>"
"        <force_pool type=\"ForcePoolObject\"/>"
"    </model>"
"</Root>";

void test_kinematic_6R()
{
	const double error = 1e-10;

	try
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_file);

		const double input_origin_p[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_v[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_a[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[6]{ -34.8372347443935, -0.179465241344625, -26.1146353297101,3.12638803734444e-13,5.6843418860808e-13, -9.9475983006414e-14 };
		const double output_origin_pm[16]{ 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0};
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ -1.53575836143848,-38.0002468240561,-16.8933419758238,-8.4241178308505,16.6881665634178,-51.5626922658449 };

		const double input_p[6]{ -0.084321840829742,0.111235847475406,0.163501201249858,0.41316722587035, -0.0861578092597486,0.229246197281016 };
		const double input_v[6]{  0.93426722257942, - 0.024823760537999, - 0.89419018046124,   0.245922301638701, - 1.23100367003297, - 0.48185561218356 };
		const double input_a[6]{ 0.70807836306709, - 0.496581922752884, - 0.159513727427361, - 0.590163055515337,   0.131806583011732, - 1.65802060177352 };
		const double input_mf[6]{ -24.6359418510515,3.06678992657553, - 13.4565070365958,   15.0336821069307,   0.786112551012351,   1.93281931696021};
		const double output_pm[16]{ 0.863013488544127, - 0.284074444773496,   0.417743256579356, - 0.137731283515364,
			0.387677110267304,   0.902605554641921, - 0.187108714132569, - 0.343275971674581,
			- 0.323904579723239,   0.323426842664891,   0.889089928341408, - 0.0474940394315194,
			0,   0,   0,   1 };
		const double output_va[6]{ -1.93242030056314,   0.500930573127293,   0.577926916892486, - 0.399682310201935, - 0.66053331463003, - 0.857440373970742};
		const double output_aa[6]{ 1.07075600145293,   0.349116022890415,   2.0925775293411, - 1.77982973680254, - 0.927893632540704,   0.0659817357654945};
		const double output_mfs[6]{ -8.44990411304192, - 54.7768126462764, - 23.2058019399381, - 18.6214939645874,   51.751313528282, - 82.047228392192};

		double result1[16], result2[16], result3[16], result4[16];

		Model m;
		m.loadXml(xml_doc);

		std::tuple<aris::Size, double> ret;

		// test inverse kinematic //
		for (auto &mot : m.motionPool())mot.activate(false);
		m.generalMotionPool().at(0).activate(true);
		m.allocateMemory();

		// in glb //
		m.generalMotionPool().at(0).setMpm(output_origin_pm);
		m.generalMotionPool().at(0).setMva(output_origin_va);
		m.generalMotionPool().at(0).setMaa(output_origin_aa);
		ret = m.kinPosInGlb(100, 1e-14);
		m.allocateMemory();
		m.kinVelInGlb();
		m.allocateMemory();
		m.kinAccInGlb();
		m.allocateMemory();
		m.dynFceInGlb();
		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).updMp();
			m.motionAtAbs(i).updMv();
			m.motionAtAbs(i).updMa();
			result1[i] = m.motionAtAbs(i).mp();
			result2[i] = m.motionAtAbs(i).mv();
			result3[i] = m.motionAtAbs(i).ma();
		}

		if (std::get<0>(ret) == 100 || !s_is_equal(6, result1, input_origin_p, error))std::cout << "model::kinPos() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, input_origin_v, error))std::cout << "model::kinVel() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, input_origin_a, error))std::cout << "model::kinAcc() 6R failed" << std::endl;
		if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_origin_mfs, error))std::cout << "model::dynFce() 6R failed" << std::endl;

		m.generalMotionPool().at(0).setMpm(output_pm);
		m.generalMotionPool().at(0).setMva(output_va);
		m.generalMotionPool().at(0).setMaa(output_aa);
		ret = m.kinPosInGlb(100, 1e-14);
		m.allocateMemory();
		m.kinVelInGlb();
		m.allocateMemory();
		m.kinAccInGlb();
		m.allocateMemory();
		m.dynFceInGlb();
		for (aris::Size i = 0; i < 6; ++i) 
		{ 
			m.motionAtAbs(i).updMp();
			m.motionAtAbs(i).updMv();
			m.motionAtAbs(i).updMa();
			result1[i] = m.motionAtAbs(i).mp();
			result2[i] = m.motionAtAbs(i).mv();
			result3[i] = m.motionAtAbs(i).ma();
		}

		if (std::get<0>(ret) == 100 || !s_is_equal(6, result1, input_p, error))std::cout << "model::kinPos() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, input_v, error))std::cout << "model::kinVel() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, input_a, error))std::cout << "model::kinAcc() 6R failed" << std::endl;
		if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_mfs, error))std::cout << "model::dynFce() 6R failed" << std::endl;

		// in prt //
		m.generalMotionPool().at(0).setMpm(output_origin_pm);
		m.generalMotionPool().at(0).setMva(output_origin_va);
		m.generalMotionPool().at(0).setMaa(output_origin_aa);
		ret = m.kinPosInPrt(100, 1e-14);
		m.allocateMemory();
		m.kinVelInPrt();
		m.allocateMemory();
		m.kinAccInPrt();
		m.allocateMemory();
		m.dynFceInPrt();
		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).updMp();
			m.motionAtAbs(i).updMv();
			m.motionAtAbs(i).updMa();
			result1[i] = m.motionAtAbs(i).mp();
			result2[i] = m.motionAtAbs(i).mv();
			result3[i] = m.motionAtAbs(i).ma();
		}

		if (std::get<0>(ret) == 100 || !s_is_equal(6, result1, input_origin_p, error))std::cout << "model::kinPosInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, input_origin_v, error))std::cout << "model::kinVelInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, input_origin_a, error))std::cout << "model::kinAccInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_origin_mfs, error))std::cout << "model::dynFceInPrt() 6R failed" << std::endl;

		m.generalMotionPool().at(0).setMpm(output_pm);
		m.generalMotionPool().at(0).setMva(output_va);
		m.generalMotionPool().at(0).setMaa(output_aa);
		ret = m.kinPosInPrt(100, 1e-14);
		m.allocateMemory();
		m.kinVelInPrt();
		m.allocateMemory();
		m.kinAccInPrt();
		m.allocateMemory();
		m.dynFceInPrt();
		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).updMp();
			m.motionAtAbs(i).updMv();
			m.motionAtAbs(i).updMa();
			result1[i] = m.motionAtAbs(i).mp();
			result2[i] = m.motionAtAbs(i).mv();
			result3[i] = m.motionAtAbs(i).ma();
		}

		if (std::get<0>(ret) == 100 || !s_is_equal(6, result1, input_p, error))std::cout << "model::kinPosInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, input_v, error))std::cout << "model::kinVelInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, input_a, error))std::cout << "model::kinAccInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_mfs, error))std::cout << "model::dynFceInPrt() 6R failed" << std::endl;

		// test forward kinematic //
		for (auto &mot : m.motionPool())mot.activate(true);
		m.generalMotionPool().at(0).activate(false);
		m.allocateMemory();

		// in glb //
		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).setMp(input_origin_p[i]);
			m.motionAtAbs(i).setMv(input_origin_v[i]);
			m.motionAtAbs(i).setMa(input_origin_a[i]);
		}
		ret = m.kinPosInGlb(100, 1e-14);
		m.allocateMemory();
		m.kinVelInGlb();
		m.allocateMemory();
		m.kinAccInGlb();
		m.allocateMemory();
		m.dynFceInGlb();
		m.generalMotionPool().at(0).updMpm();
		m.generalMotionPool().at(0).getMpm(result1);
		m.generalMotionPool().at(0).updMvs();
		m.generalMotionPool().at(0).getMva(result2);
		m.generalMotionPool().at(0).updMas();
		m.generalMotionPool().at(0).getMaa(result3);
		for (int i = 0; i < 6; ++i)result4[i] = m.motionAtAbs(i).mf();

		if (std::get<0>(ret) == 100 || !s_is_equal(16, result1, output_origin_pm, error))std::cout << "model::kinPos() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, output_origin_va, error))std::cout << "model::kinVel() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, output_origin_aa, error))std::cout << "model::kinAcc() 6R failed" << std::endl;
		if (!s_is_equal(6, result4, input_origin_mf, error))std::cout << "model::dynFce() 6R failed" << std::endl;

		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).setMp(input_p[i]);
			m.motionAtAbs(i).setMv(input_v[i]);
			m.motionAtAbs(i).setMa(input_a[i]);
		}
		ret = m.kinPosInGlb(100, 1e-14);
		m.allocateMemory();
		m.kinVelInGlb();
		m.allocateMemory();
		m.kinAccInGlb();
		m.allocateMemory();
		m.dynFceInGlb();
		m.generalMotionPool().at(0).updMpm();
		m.generalMotionPool().at(0).getMpm(result1);
		m.generalMotionPool().at(0).updMvs();
		m.generalMotionPool().at(0).getMva(result2);
		m.generalMotionPool().at(0).updMas();
		m.generalMotionPool().at(0).getMaa(result3);
		for (int i = 0; i < 6; ++i)result4[i] = m.motionAtAbs(i).mf();
		if (std::get<0>(ret) == 100 || !s_is_equal(16, result1, output_pm, error))std::cout << "model::kinPos() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, output_va, error))std::cout << "model::kinVel() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, output_aa, error))std::cout << "model::kinAcc() 6R failed" << std::endl;
		if (!s_is_equal(6, result4, input_mf, error))std::cout << "model::dynFce() 6R failed" << std::endl;

		// in prt //
		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).setMp(input_origin_p[i]);
			m.motionAtAbs(i).setMv(input_origin_v[i]);
			m.motionAtAbs(i).setMa(input_origin_a[i]);
		}
		ret = m.kinPosInPrt(100, 1e-14);
		m.allocateMemory();
		m.kinVelInPrt();
		m.allocateMemory();
		m.kinAccInPrt();
		m.allocateMemory();
		m.dynFceInPrt();
		m.generalMotionPool().at(0).updMpm();
		m.generalMotionPool().at(0).getMpm(result1);
		m.generalMotionPool().at(0).updMvs();
		m.generalMotionPool().at(0).getMva(result2);
		m.generalMotionPool().at(0).updMas();
		m.generalMotionPool().at(0).getMaa(result3);
		for (int i = 0; i < 6; ++i)result4[i] = m.motionAtAbs(i).mf();

		if (std::get<0>(ret) == 100 || !s_is_equal(16, result1, output_origin_pm, error))std::cout << "model::kinPosInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, output_origin_va, error))std::cout << "model::kinVelInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, output_origin_aa, error))std::cout << "model::kinAccInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result4, input_origin_mf, error))std::cout << "model::dynFceInPrt() 6R failed" << std::endl;

		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).setMp(input_p[i]);
			m.motionAtAbs(i).setMv(input_v[i]);
			m.motionAtAbs(i).setMa(input_a[i]);
		}
		ret = m.kinPosInPrt(100, 1e-14);
		m.allocateMemory();
		m.kinVelInPrt();
		m.allocateMemory();
		m.kinAccInPrt();
		m.allocateMemory();
		m.dynFceInPrt();
		m.generalMotionPool().at(0).updMpm();
		m.generalMotionPool().at(0).getMpm(result1);
		m.generalMotionPool().at(0).updMvs();
		m.generalMotionPool().at(0).getMva(result2);
		m.generalMotionPool().at(0).updMas();
		m.generalMotionPool().at(0).getMaa(result3);
		for (int i = 0; i < 6; ++i)result4[i] = m.motionAtAbs(i).mf();
		if (std::get<0>(ret) == 100 || !s_is_equal(16, result1, output_pm, error))std::cout << "model::kinPosInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, output_va, error))std::cout << "model::kinVelInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, output_aa, error))std::cout << "model::kinAccInPrt() 6R failed" << std::endl;
		if (!s_is_equal(6, result4, input_mf, error))std::cout << "model::dynFceInPrt() 6R failed" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_simulation_6R()
{
}


void test_model_6R()
{
	std::cout << std::endl << "-----------------test model 6R---------------------" << std::endl;

	test_kinematic_6R();
	test_simulation_6R();

	std::cout << "-----------------test model 6R finished------------" << std::endl << std::endl;
}

