#include "test_dynamic_model_3R.h"
#include <iostream>
#include <aris.h>

using namespace aris::dynamic;

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"utf-8\"?>"
"<root>"
"    <model>"
"        <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"        <variable_pool type=\"VariablePoolObject\" default_child_type=\"Matrix\">"
"            <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"            <Mot_friction type=\"MatrixVariable\">{20, 30, 560}</Mot_friction>"
"        </variable_pool>"
"        <akima_pool type=\"AkimaPoolObject\" default_child_type=\"Akima\">"
"        </akima_pool>"
"        <part_pool type=\"PartPoolObject\" default_child_type=\"Part\">"
"            <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </ground>"
"            <part1 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <r1i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r2j pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolObject\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t\"/>"
"                </geometry_pool>"
"            </part1>"
"            <part2 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,0,0,PI/2,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <r2i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r3j pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolObject\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part2.x_t\"/>"
"                </geometry_pool>"
"            </part2>"
"            <part3 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0.2,0.5,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <r3i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <ee pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolObject\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part3.x_t\"/>"
"                </geometry_pool>"
"            </part3>"
"            <part4 active=\"false\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0.2,0.5,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"                <geometry_pool type=\"GeometryPoolObject\">"
"                    <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part3.x_t\"/>"
"                </geometry_pool>"
"            </part4>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolObject\">"
"            <r1 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\"/>"
"            <r2 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\"/>"
"            <r3 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"            <r4 active=\"false\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolObject\" default_child_type=\"Motion\">"
"            <m1 active=\"true\" slave_id=\"0\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"            <m2 active=\"true\" slave_id=\"1\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"            <m3 active=\"true\" slave_id=\"2\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolObject\" default_child_type=\"GeneralMotion\"/>"
"    </model>"
"</root>";

void test_kinematic_3R()
{
	const double error = 1e-10;

	try
	{
		aris::core::XmlDocument xml_doc;
		auto a = xml_doc.Parse(xml_file);

		Model m;
		m.loadXml(xml_doc);

		// test kinematic //
		m.allocateMemory();
		m.motionAtAbs(0).setMp(0.585);
		m.motionAtAbs(0).setMv(0.235);
		m.motionAtAbs(0).setMa(-1.567);

		m.motionAtAbs(1).setMp(0.685);
		m.motionAtAbs(1).setMv(0.235);
		m.motionAtAbs(1).setMa(-1.567);

		m.motionAtAbs(2).setMp(-0.312);
		m.motionAtAbs(2).setMv(0.235);
		m.motionAtAbs(2).setMa(-1.567);

		const double ee_pq[7]{ 1.70515017988957 , 2.32534312862588, 0.0 , 0.0 , 0.0 , 0.460891949876968 , 0.887456258380438 };
		const double ee_va[6]{ -1.1553844949236325 , 0.7406601913177889 , 0.0 , 0.0 , 0.0 , 0.705 };
		const double ee_aa[6]{ 7.3068444301678799 ,-5.5868499482603751 , 0.0 , 0.0 , 0.0 ,-4.701 };
		double result1[16], result2[16], result3[16];

		auto ret = m.kinPos(100, 1e-12);
		m.kinVel();
		m.kinAcc();
		m.dynFce();

		m.partPool().at(3).markerPool().findByName("ee")->getPq(result1);
		m.partPool().at(3).markerPool().findByName("ee")->getVa(result2);
		m.partPool().at(3).markerPool().findByName("ee")->getAa(result3);

		if (std::get<0>(ret) == 100 || !s_is_equal(7, ee_pq, result1, error))std::cout << "model::kinPos() 3R failed" << std::endl;
		if (!s_is_equal(6, result2, ee_va, error))std::cout << "\"model:kinVel() 3R\" failed" << std::endl;
		if (!s_is_equal(6, result3, ee_aa, error))std::cout << "\"model:kinAcc() 3R\" failed" << std::endl;

		// benchmark efficiency //
		auto bench = aris::core::benchmark(100, [&m]()
		{
			static int count{ 0 };

			if (count % 2)
			{
				m.motionAtAbs(0).setMp(0.585);
				m.motionAtAbs(1).setMp(0.685);
				m.motionAtAbs(2).setMp(-0.312);
			}
			else
			{
				m.motionAtAbs(0).setMp(0.0);
				m.motionAtAbs(1).setMp(0.0);
				m.motionAtAbs(2).setMp(0.0);
			}

			auto ret = m.kinPos(100);
			if (count++<2)std::cout << "benchmark computation finished, spend " << std::get<0>(ret) << " count with error " << std::get<1>(ret) << std::endl;
		});

		std::cout << "computational time:" << bench << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_simulation_3R()
{
	try
	{
		aris::core::XmlDocument xml_doc;
		auto a = xml_doc.Parse(xml_file);

		Model m;
		m.loadXml(xml_doc);
		m.akimaPool().add<Akima>(std::string("m1_akima"), std::list<double>{ 0.0, 1.0, 2.0, 3.0, 4.0 }, std::list<double>{0.0, 1.0, 2.0, 3.0, 4.0 });
		m.akimaPool().add<Akima>(std::string("m2_akima"), std::list<double>{ 0.0, 1.0, 2.0, 3.0, 4.0 }, std::list<double>{0.0, 1.0, 2.0, 3.0, 4.0 });
		m.akimaPool().add<Akima>(std::string("m3_akima"), std::list<double>{ 0.0, 1.0, 2.0, 3.0, 4.0 }, std::list<double>{0.0, 1.0, 2.0, 3.0, 4.0 });

		aris::dynamic::PlanParamBase p;
		aris::dynamic::PlanFunc f = [](aris::dynamic::Model &m, const aris::dynamic::PlanParamBase &p)
		{
			double pe2[6]{ 0,0,0,0,0,0 };
			pe2[5] = p.count_*0.001 / 3 * PI / 3 + PI / 2;
			auto &r2j = *m.partPool().findByName("part1")->markerPool().findByName("r2j");
			m.partPool().findByName("part2")->setPe(r2j, pe2, "123");

			double pe3[6]{ 0,0,0,0,0,0 };
			pe3[5] = -p.count_*0.001 / 3 * PI / 3 - PI / 2;
			auto &r3j = *m.partPool().findByName("part2")->markerPool().findByName("r3j");
			m.partPool().findByName("part3")->setPe(r3j, pe3, "123");

			m.motionAtAbs(0).updMp();
			m.motionAtAbs(1).updMp();
			m.motionAtAbs(2).updMp();

			return 3000 - p.count_;
		};

		m.saveDynEle("before");
		m.simKin(f, p);
		m.loadDynEle("before");
		//m.saveAdams("C:\\aris\\robot\\resource\\test.cmd");

		std::cout << "test simulation finished, please check \"C:\\aris\\robot\\resource\\test.cmd\"" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}


void test_model_3R()
{
	std::cout << std::endl << "-----------------test model 3R---------------------" << std::endl;
	
	test_kinematic_3R();
	test_simulation_3R();

	std::cout << "-----------------test model 3R finished------------" << std::endl << std::endl;
}

