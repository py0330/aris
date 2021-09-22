#include "aris.hpp"

int main()
{
	{
		aris::dynamic::Model m;
		//aris::core::fromXmlFile(m, "C:\\Users\\py033\\Desktop\\moveSine_log\\model.xml");
		auto &clb = m.calibratorPool().add<aris::dynamic::Calibrator>();
		clb.setDataIndex(0, 1, 2, 3);
		clb.setFilterWindowSize(30);
		clb.setTorqueConstant({
			-1.27 * 80 / 1000,
			1.27 * 121 / 1000,
			0.64 * 57.9545 / 1000,
			0.318 * 122.4 / 1000,
			0.318 * 51 / 1000,
			-0.318 * 50 / 1000 });
		clb.setTorqueWeight({
			1.27 * 80,
			1.27 * 121,
			0.64 * 57.9545,
			0.318 * 122.4,
			0.318 * 51,
			0.318 * 50 });
		clb.setVelocityRatio({
			1.0,
			1.0,
			1.0,
			1.0,
			1.0,
			1.0,
			});

		m.init();

		std::cout << aris::core::toXmlString(m) << std::endl;

		clb.clbFiles({
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\calib_dyn_par.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\1\\calib_dyn_par1.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\1\\calib_dyn_par2.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\1\\calib_dyn_par3.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\1\\calib_dyn_par4.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\1\\calib_dyn_par5.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\1\\calib_dyn_par6.txt",
			});

		std::cout << "verify" << std::endl;
		clb.verifyFiles({
			"C:\\Users\\py033\\Desktop\\moveSine_log\\calib_dyn_par.txt",
			//"C:\\Users\\py033\\Desktop\\data2.txt"
			});

	}
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

	param.iv_vec = { 
	{   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   3.63013949732480,   0.00000000000000,   0.00000000000000,   0.00000000000000},
	{	0.00000000000000, - 0.09564115551277,   0.00000000000000,   0.00000000000000,   0.00000000000000,   4.18829437485302,   0.00000000000000,   0.11663124404160,   0.22345797254906,   0.02429434893557},
	{	0.00000000000000, - 0.04148965464841,   0.00000000000000,   5.27181632349865,   0.00000000000000,   4.25082897142102, - 4.25573556523412, - 0.03037719611212, - 0.05558794400655,   0.72021637991932},
	{	0.00000000000000, - 0.02996286586369,   1.18450369449036,   0.00000000000000,   0.00000000000000, - 4.03900369361803,   4.04535968986888, - 0.01685400135128, - 0.04226166649485, - 0.77188579912113},
	{	0.00000000000000,   0.00283075488860,   0.00000000000000,   1.69767402171386,   6.87503687557822,   6.83113044237905,   0.05980263907514, - 0.03939761368391,   0.04148834100736,   0.03416762301695},
	{	0.00000000000000, - 0.00096907974208, - 0.24592781267872,   0.00000000000000,   0.23891256219112,   0.26361900516083,   0.00000000000000, - 0.01350902820507,   0.01693500970538,   0.23406985475929},
	{	0.00000000000000, - 0.00675056216385, - 0.00721088285743,   0.12441295392378, - 0.00645737318402,   0.00000000000000,   0.01294125405470, - 0.01116651698841, - 0.00035225594900,   0.01239126899263},
	};

	param.mot_frc_vec = {
	{ 12.49218385392151,   32.65199721297471,   0.00000000000000 },
	{ 16.10888271207234,   38.14836057950608,   0.00000000000000 },
	{ 7.85833919758804,   16.30289666290987,   1.40436083397210 },
	{ 4.30844611880665,   5.37600812849376,   0.07827512100895 },
	{ 3.49239301279372,   4.03969890831296,   0.23640992856187 },
	{ 3.15056837923699,   3.84933585917708,   0.23383617156822 },
	{ 3.39184737968181,   5.19170732055650,   0.31458578971719 },
	};

	
		

	auto m = aris::dynamic::createModelSevenAxis2(param);
	auto &clb = m->calibratorPool().add<aris::dynamic::Calibrator>();
	clb.setDataIndex(0, 1, 2, 3);
	clb.setFilterWindowSize(30);
	clb.setTorqueConstant({ 
		-1.27 * 80 / 1000,
		1.27 * 121 / 1000,
		0.64 * 57.9545 / 1000,
		0.318 * 122.4 / 1000,
		0.318 * 51 / 1000,
		-0.318 * 50 / 1000 });
	clb.setTorqueWeight({
		-1.27 * 80 / 1000,
		1.27 * 121 / 1000,
		0.64 * 57.9545 / 1000,
		0.318 * 122.4 / 1000,
		0.318 * 51 / 1000,
		-0.318 * 50 / 1000 });
	clb.setVelocityRatio({
		1.0,
		1.0,
		1.0,
		1.0,
		1.0,
		1.0,
		});

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
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine9.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine10.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine11.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine12.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine13.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine14.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine15.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine16.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine17.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine18.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine19.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine20.txt",
			//"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data0.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data1.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data2.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data3.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data4.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data5.txt",
			"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\data6.txt",
		});


		aris::dynamic::dsp(1, 10, m->partPool()[1].prtIv());
		aris::dynamic::dsp(1, 10, m->partPool()[2].prtIv());
		aris::dynamic::dsp(1, 10, m->partPool()[3].prtIv());
		aris::dynamic::dsp(1, 10, m->partPool()[4].prtIv());
		aris::dynamic::dsp(1, 10, m->partPool()[5].prtIv());
		aris::dynamic::dsp(1, 10, m->partPool()[6].prtIv());
		aris::dynamic::dsp(1, 10, m->partPool()[7].prtIv());

		aris::dynamic::dsp(1, 3, m->motionPool()[0].frcCoe());
		aris::dynamic::dsp(1, 3, m->motionPool()[1].frcCoe());
		aris::dynamic::dsp(1, 3, m->motionPool()[2].frcCoe());
		aris::dynamic::dsp(1, 3, m->motionPool()[3].frcCoe());
		aris::dynamic::dsp(1, 3, m->motionPool()[4].frcCoe());
		aris::dynamic::dsp(1, 3, m->motionPool()[5].frcCoe());
		aris::dynamic::dsp(1, 3, m->motionPool()[6].frcCoe());

		//clb.clbFiles({
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--26.txt",
		//	"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--27.txt",
		//	});


		//0.00000000000000   0.00000000000000   0.00000000000000   0.00000000000000   0.00000000000000   0.00000000000000   3.63013949732480   0.00000000000000   0.00000000000000   0.00000000000000
		//	0.00000000000000 - 0.09564115551277   0.00000000000000   0.00000000000000   0.00000000000000   4.18829437485302   0.00000000000000   0.11663124404160   0.22345797254906   0.02429434893557
		//	0.00000000000000 - 0.04148965464841   0.00000000000000   5.27181632349865   0.00000000000000   4.25082897142102 - 4.25573556523412 - 0.03037719611212 - 0.05558794400655   0.72021637991932
		//	0.00000000000000 - 0.02996286586369   1.18450369449036   0.00000000000000   0.00000000000000 - 4.03900369361803   4.04535968986888 - 0.01685400135128 - 0.04226166649485 - 0.77188579912113
		//	0.00000000000000   0.00283075488860   0.00000000000000   1.69767402171386   6.87503687557822   6.83113044237905   0.05980263907514 - 0.03939761368391   0.04148834100736   0.03416762301695
		//	0.00000000000000 - 0.00096907974208 - 0.24592781267872   0.00000000000000   0.23891256219112   0.26361900516083   0.00000000000000 - 0.01350902820507   0.01693500970538   0.23406985475929
		//	0.00000000000000 - 0.00675056216385 - 0.00721088285743   0.12441295392378 - 0.00645737318402   0.00000000000000   0.01294125405470 - 0.01116651698841 - 0.00035225594900   0.01239126899263

		//0.00000000000000 - 0.00675056216385 - 0.00721088285743
		//	0.12441295392378 - 0.00645737318402   0.00000000000000
		//	0.01294125405470 - 0.01116651698841 - 0.00035225594900
		//	0.01239126899263   12.49218385392151   32.65199721297471
		//	0.00000000000000   16.10888271207233   38.14836057950610
		//	0.00000000000000   7.85833919758803   16.30289666290988
		//	1.40436083397210   4.30844611880665   5.37600812849376


		std::cout << "verify" << std::endl;
		//clb.verifyFiles({
		//	"C:\\Users\\py033\\Desktop\\moveSine_log\\moveSine_log\\movesine1.txt",
		//	//"C:\\Users\\py033\\Desktop\\data2.txt"
		//	});


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

