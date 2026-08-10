#include <aris.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

class Test
{
public:

	auto test_trajectory_aj(aris::server::ControlServer& cs)->void;
	auto test_trajectory_l(aris::server::ControlServer& cs)->void;
	auto pd()->aris::plan::PlannerDispacher*;
	auto init() ->void;
	auto writeFile(std::string path)->void;
	auto run()->int;
	Test();
	~Test();

private:
	struct Imp;
	std::unique_ptr<Imp> imp_;
};

double z0[2] =   { 0,    0 };
double z10[2] =  { 0.01, 10 * aris::PI / 180.0 };
double z20[2] =  { 0.02, 20 * aris::PI / 180.0 };
double z50[2] =  { 0.05, 30 * aris::PI / 180.0 };
double z80[2] =  { 0.08, 40 * aris::PI / 180.0 };
double z100[2] = { 0.10, 50 * aris::PI / 180.0 };
double z[2]    = { 0.10, 50 * aris::PI / 180.0 };

double v0[3] =     { 0,   0,    0 };
double v10[3] =    { 3,   0.01, aris::PI };
double v50[3] =    { 7,   0.05, aris::PI };
double v100[3] =   { 10,  0.1,  aris::PI };
double v500[3] =   { 50,  0.5,  aris::PI };
double v1000[3] =  { 100, 1,    aris::PI };
double v2000[3] =  { 100, 2,    aris::PI };
double v10000[3] = { 100, 10,   aris::PI };
double v[3] =      { 50, 0.05,  aris::PI };

double time_zone = 0.1;

struct Test::Imp
{

	std::unique_ptr<aris::plan::PlannerDispacher> pd{new aris::plan::PlannerDispacher};

	std::vector<int> prepare_id, cur_exe_id, cur_exe_count, cmd_type;
	int insert_pe_id = 0;

	std::vector<std::vector<double>> pes;// 笛卡尔位姿
	std::vector<std::vector<double>> ajs;// 轴关节角度
	std::vector <double> vel, acc, jerk, zone;
	double max_line_vel = 6;
	double max_angle_vel = 720 * aris::PI / 180.0;
	std::vector<double> rokae_max_vel = { 400,400,410,440,330,700};
	std::vector<double> rokae_max_acc = { 2000,2000,2050,2200,1650,3500 };

	std::vector<double> scara_max_vel = { 9.42477788067463251 ,5.88175952923583534 ,1.68700000000000006,26.17993855742953357 };
	std::vector<double> scara_max_acc = { 188.49555761349265026,117.6351905847167103,33.74000000000000199,523.5987711485906857 };
	std::vector<double> puma_max_vel{ 5.8904862254808625,4.8432886742842651, 5.0963614158234423 , 6.8067840827778845 ,7.8539816339744828 ,12.5663706143591725 };
	std::vector<double> puma_max_acc{ 11.780972 ,  9.686577 ,  10.192723 ,  13.613568 ,  15.707963  , 25.132741 };

	// 
	std::vector<double> vec;
	int line_num = 0;

};

Test::Test() : imp_(new Imp)
{

	aris::dynamic::s_nv(6, aris::PI / 180.0, imp_->rokae_max_vel.data());
	aris::dynamic::s_nv(6, aris::PI / 180.0, imp_->rokae_max_acc.data());

}
Test::~Test() { }
auto Test::init() ->void
{
	imp_->line_num = 0;
	imp_->vec.clear();
}
auto Test::writeFile(std::string path)->void
{
	aris::dynamic::dlmwrite(imp_->vec.size() / 6, 6,imp_->vec.data(), path.data());
	init();
}
auto Test::pd()->aris::plan::PlannerDispacher* {return imp_->pd.get();}
auto Test::run()->int {
	auto move_aj_and_copy_data = [&]()->int
	{
		// m++;
		imp_->line_num++;
		imp_->vec.resize(imp_->line_num * 6, 0.0);
		auto ret = imp_->pd->getNextInput(0,imp_->vec.data() + 6 * (imp_->line_num - 1));
		//aris::dynamic::dsp(1,6,imp_->vec.data() + 6 * (imp_->line_num - 1));
		
		if (ret == 0) {
			std::cout << "aj trajectory finished" << std::endl;
		}

		static int cnt = 0;
		if (cnt++ % 100 == 0)
			std::cout << "ret:" << ret << std::endl;
		return ret;
	};

	while (move_aj_and_copy_data()) {};
	return 0;
}
auto Test::test_trajectory_aj(aris::server::ControlServer& cs)->void {

	std::cout << "-----------------test aj trajectory start---------------" << std::endl;
	

	double end_pos[6] {0,-0.53,1.98,0.23,-1.85,0};
	double vels[6] {0.26179912599976163,0.26179912599976163,0.26179912599976163,0.31415895119971399,0.31415895119971399,0.31415895119971399};
	double accs[6] {5.5192101840043515,5.5192101840043515,5.5192101840043515,5.5192101840043515,5.5192101840043515,5.5192101840043515};
	double jerks[6] {174.53275066650772,174.53275066650772,174.53275066650772,174.53275066650772,174.53275066650772,174.53275066650772};
	double zones[6] {0,0,0,0,0,0};
	
	auto ch_lock_ret = imp_->pd->tryLockChanel(0,{0});
	imp_->pd->insertMoveAbsJPos(0,end_pos,vels,accs,jerks,zones,time_zone);

	imp_->pd->updateInsertPos(0);





	//imp_->pd->releaseChanel(0);

	

	std::cout << "-----------------test aj trajectory finished------------" << std::endl;
}
auto Test::test_trajectory_l(aris::server::ControlServer& cs)->void {

	std::cout << "-----------------test l trajectory start---------------" << std::endl;


	double end_pos[6] {0.81,0.104,0.712,6.28,0.106,2.85};
	double vels[2] {0.1,3.14};
	double accs[2] {1.93,15.7};
	double jerks[2] {150,314.7};
	double zones[2] {0.01,0.17};
	std::string tool = "UrModel.L6.tool0";
	std::string wobj = "UrModel.ground.wobj0";
	auto ch_lock_ret = imp_->pd->tryLockChanel(0,{0});
	imp_->pd->insertLinePos(0,tool,wobj,end_pos,vels,accs,jerks,zones,time_zone);

	imp_->pd->updateInsertPos(0);



	std::cout << "-----------------test l trajectory finished------------" << std::endl;
}



int main() {
	auto& cs = aris::server::ControlServer::instance();
	//aris::core::fromXmlFile(cs, "..\\puma-ext.xml");
	//aris::core::fromXmlFile(cs, "..\\7-900.xml");
	//aris::core::fromXmlFile(cs, "..\\astun.xml");
	//aris::core::fromXmlFile(cs, "..\\lansi_puma.xml");
	//aris::core::fromXmlFile(cs, "..\\scara-800.xml");
	
	aris::core::fromXmlFile(cs, "/Users/panyang/Desktop/test_aris_plan/ur.xml");
	// aris::core::fromXmlFile(cs, "..\\rokae.xml");
	//aris::core::fromXmlFile(cs, "..\\astun-er35b.xml");

	cs.init();
	cs.start();
	
	Test t;
	t.init();
	auto& mm = dynamic_cast<aris::dynamic::MultiModel&>(cs.model());
	

	t.pd()->setModel(mm);
	t.pd()->setChanelSize(2); 
	t.pd()->setDt(2*1e-3);
	t.pd()->init();
	t.pd()->setTargetSpeedRatio(0, 1);

	t.test_trajectory_aj(cs);
	t.test_trajectory_l(cs);
	t.test_trajectory_aj(cs);
	
	t.run();

	// 数据写入文档
	t.writeFile(std::string("./pos.txt"));



    return 0;
}