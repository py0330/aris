#include "aris.hpp"

int main()
{
	auto m = aris::dynamic::createModelRokaeXB4();

	std::cout << m->xmlString() << std::endl;

	auto &clb = m->calibratorPool().add<aris::dynamic::Calibrator>();

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
		clb.clbFiles({
			"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--26.txt",
			"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--27.txt",
			});


		std::cout << "verify" << std::endl;

		clb.verifyFiles({
			"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--26.txt",
			"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-17--20-39-37--27.txt",
			"C:\\Users\\py033\\Desktop\\data_2018-12-09\\rt_log--2018-12-09--17-11-03--48.txt",
			"C:\\Users\\py033\\Desktop\\rokae_data\\j1_rt_log--2018-09-28--11-41-12--6.txt",
			"C:\\Users\\py033\\Desktop\\rokae_data\\j2_rt_log--2018-09-28--11-41-12--24.txt",
			"C:\\Users\\py033\\Desktop\\rokae_data\\j3_rt_log--2018-09-28--11-41-12--26.txt" ,
			"C:\\Users\\py033\\Desktop\\rokae_data\\j4_rt_log--2018-09-28--11-41-12--37.txt" ,
			"C:\\Users\\py033\\Desktop\\rokae_data\\j5_rt_log--2018-09-28--11-41-12--38.txt" ,
			"C:\\Users\\py033\\Desktop\\rokae_data\\j6_rt_log--2018-09-28--11-41-12--39.txt"
			});
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}


	char a;
	std::cin >> a;

	return 0;
}

