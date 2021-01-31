#include "aris.hpp"

int main()
{
	//auto m = aris::robot::createModelRokaeXB4();

	//auto &clb = m->calibratorPool().add<aris::dynamic::Calibrator>();

	try
	{
		/*clb.clbFiles({ 
			"C:\\Users\\py033\\Desktop\\rokae_data\\j1_rt_log--2018-09-28--11-41-12--6.txt",
			"C:\\Users\\py033\\Desktop\\rokae_data\\j2_rt_log--2018-09-28--11-41-12--24.txt",
			"C:\\Users\\py033\\Desktop\\rokae_data\\j3_rt_log--2018-09-28--11-41-12--26.txt" ,
			"C:\\Users\\py033\\Desktop\\rokae_data\\j4_rt_log--2018-09-28--11-41-12--37.txt" ,
			"C:\\Users\\py033\\Desktop\\rokae_data\\j5_rt_log--2018-09-28--11-41-12--38.txt" ,
			"C:\\Users\\py033\\Desktop\\rokae_data\\j6_rt_log--2018-09-28--11-41-12--39.txt" 
			});*/
		//clb.clbFiles({
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--26.txt",
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--27.txt",
		//	});


		std::cout << "verify" << std::endl;

		//clb.verifyFiles({
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--26.txt",
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--27.txt",
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-09--17-11-03--48.txt",
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j1_rt_log--2018-09-28--11-41-12--6.txt",
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j2_rt_log--2018-09-28--11-41-12--24.txt",
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j3_rt_log--2018-09-28--11-41-12--26.txt" ,
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j4_rt_log--2018-09-28--11-41-12--37.txt" ,
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j5_rt_log--2018-09-28--11-41-12--38.txt" ,
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j6_rt_log--2018-09-28--11-41-12--39.txt"
		//	});
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}


	aris::dynamic::SevenAxisParam2 param;

	//param.d1 = 0;
	//param.a2 = 0.1;
	//param.d3 = 0.330;
	//param.d5 = 0.320;
	//param.tool0_pe[2] = 0;


	param.d1 = 0.203;
	param.a2 = -0.138;
	param.d3 = 0.450;
	param.d5 = 0.300;
	param.tool0_pe[2] = 0.1048;

	auto m = aris::dynamic::createModelSevenAxis2(param);
	auto &clb = m->calibratorPool().add<aris::dynamic::Calibrator>();
	clb.setDataIndex(0, 1, 2, 3);
	clb.setFilterWindowSize(30);
	clb.setTorqueConstant({ 
		0.7 * 160 / 1000,
		-0.7 * 160 / 1000,
		0.7 * 100 / 1000,
		0.45 * 100 / 1000,
		0.45 * 100 / 1000,
		0.45 * 100 / 1000,
		0.45 * 100 / 1000 });

	clb.setVelocityRatio({
		1.0 / 160 / 6.13,
		1.0 / 160 / 6.13,
		1.0 / 100 / 6.13,
		1.0 / 100 / 6.13,
		1.0 / 100 / 6.13,
		1.0 / 100 / 6.13,
		1.0 / 100 / 6.13, });

	m->init();
	
	try
	{
		clb.clbFiles({
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine1.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine2.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine3.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine4.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine5.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine6.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine7.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine8.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine9.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine10.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine11.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine12.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine13.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine14.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine15.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine16.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine17.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine18.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine19.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine20.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data0.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data1.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data2.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data3.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data4.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data5.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data6.txt",
		});
		//clb.clbFiles({
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--26.txt",
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--27.txt",
		//	});


		std::cout << "verify" << std::endl;
		clb.verifyFiles({
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data1.txt",
			"C:\\Users\\py033\\Desktop\\data1.txt"
			});


		//clb.verifyFiles({
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--26.txt",
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--27.txt",
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-09--17-11-03--48.txt",
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j1_rt_log--2018-09-28--11-41-12--6.txt",
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j2_rt_log--2018-09-28--11-41-12--24.txt",
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j3_rt_log--2018-09-28--11-41-12--26.txt" ,
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j4_rt_log--2018-09-28--11-41-12--37.txt" ,
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j5_rt_log--2018-09-28--11-41-12--38.txt" ,
		//	"C:\\Users\\py033\\Desktop\\rokae_data\\j6_rt_log--2018-09-28--11-41-12--39.txt"
		//	});
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}


	char a;
	std::cin >> a;

	return 0;
}

