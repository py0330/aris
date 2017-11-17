#include "test_dynamic_model_multi.h"
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
"        <ds type=\"DiagSolver\"/>"
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
		auto &ds = static_cast<DiagSolver&>(*m.solverPool().findByName("ds"));
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

void test_model_multi()
{
	std::cout << std::endl << "-----------------test model---------------------" << std::endl;
	test_solver_multi();
	std::cout << "-----------------test model finished------------" << std::endl << std::endl;
}

