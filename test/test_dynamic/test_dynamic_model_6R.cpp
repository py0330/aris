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
		const double output_origin_pm[16]{ 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0};
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };

		const double input_p[6]{ -0.864321840829742,0.111235847475406,0.635012012498587,0.41316722587035, -0.0861578092597486,0.229246197281016 };
		const double input_v[6]{ 76.9342672257942, - 7.0248760537999, - 89.419018046124,   24.5922301638701, - 8.23100367003297, - 7.48185561218356 };
		const double input_a[6]{ 708.07836306709, - 496.581922752884, - 1595.13727427361, - 59.0163055515337,   1318.06583011732, - 165.802060177352 };
		const double output_pm[16]{ 0.969061486621173, - 0.122712022612269,   0.214153203697654, - 0.2,
			0.146459319092386,   0.98427640122227, - 0.0987402341901776, - 0.5,
			- 0.198669330795061,   0.127050090528668,   0.971796671890833,   0.1,
			0,0,0,1	};
		const double output_va[6]{ 0.35,0.67,0.12,1.85,-6.52,0.66 };
		const double output_aa[6]{ 0.111,0.222,0.333,0.444,0.555,0.666 };

		double result1[16], result2[16], result3[16];

		Model m;
		m.loadXml(xml_doc);

		std::tuple<aris::Size, double> ret;

		// test inverse kinematic //
		for (auto &mot : m.motionPool())mot.activate(false);
		m.generalMotionPool().at(0).activate(true);
		m.allocateMemory();

		m.generalMotionPool().at(0).setMpm(output_origin_pm);
		m.generalMotionPool().at(0).setMva(output_origin_va);
		m.generalMotionPool().at(0).setMaa(output_origin_aa);
		ret = m.kinPos(100, 1e-12);
		m.kinVel();
		m.kinAcc();
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

		
		m.generalMotionPool().at(0).setMpm(output_pm);
		m.generalMotionPool().at(0).setMva(output_va);
		m.generalMotionPool().at(0).setMaa(output_aa);
		ret = m.kinPos(100, 1e-12);
		m.kinVel();
		m.kinAcc();
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
		if (!s_is_equal(6, result2, input_v, 1e-9))std::cout << "model::kinVel() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, input_a, 1e-7))std::cout << "model::kinAcc() 6R failed" << std::endl;
		
		// test forward kinematic //
		for (auto &mot : m.motionPool())mot.activate(true);
		m.generalMotionPool().at(0).activate(false);
		m.allocateMemory();

		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).setMp(input_origin_p[i]);
			m.motionAtAbs(i).setMv(input_origin_v[i]);
			m.motionAtAbs(i).setMa(input_origin_a[i]);
		}
		ret = m.kinPos(100, 1e-12);
		m.kinVel();
		m.kinAcc();
		m.generalMotionPool().at(0).updMp();
		m.generalMotionPool().at(0).getMpm(result1);
		m.generalMotionPool().at(0).updMv();
		m.generalMotionPool().at(0).getMva(result2);
		m.generalMotionPool().at(0).updMa();
		m.generalMotionPool().at(0).getMaa(result3);

		if (std::get<0>(ret) == 100 || !s_is_equal(16, result1, output_origin_pm, error))std::cout << "model::kinPos() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, output_origin_va, error))std::cout << "model::kinVel() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, output_origin_aa, error))std::cout << "model::kinAcc() 6R failed" << std::endl;

		for (aris::Size i = 0; i < 6; ++i)
		{
			m.motionAtAbs(i).setMp(input_p[i]);
			m.motionAtAbs(i).setMv(input_v[i]);
			m.motionAtAbs(i).setMa(input_a[i]);
		}
		ret = m.kinPos(100, 1e-12);
		m.kinVel();
		m.kinAcc();
		m.generalMotionPool().at(0).updMp();
		m.generalMotionPool().at(0).getMpm(result1);
		m.generalMotionPool().at(0).updMv();
		m.generalMotionPool().at(0).getMva(result2);
		m.generalMotionPool().at(0).updMa();
		m.generalMotionPool().at(0).getMaa(result3);

		if (std::get<0>(ret) == 100 || !s_is_equal(16, result1, output_pm, error))std::cout << "model::kinPos() 6R failed" << std::endl;
		if (!s_is_equal(6, result2, output_va, 1e-9))std::cout << "model::kinVel() 6R failed" << std::endl;
		if (!s_is_equal(6, result3, output_aa, 1e-7))std::cout << "model::kinAcc() 6R failed" << std::endl;
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

