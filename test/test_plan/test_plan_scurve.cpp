#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;




double drand() {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<> dis(0.0, 1.0);

	return dis(gen);
}

void test_s_curve_vaj(const std::list<SCurveNode> &scurve) {
	std::vector<LargeNum> last_last_last_p, last_last_p, last_p, p;
	std::vector<double> v, a, j;

	last_last_last_p.resize(scurve.begin()->params_.size(), 0.0);
	last_last_p.resize(scurve.begin()->params_.size(), 0.0);
	last_p.resize(scurve.begin()->params_.size(), 0.0);
	p.resize(scurve.begin()->params_.size(), 0.0);
	v.resize(scurve.begin()->params_.size(), 0.0);
	a.resize(scurve.begin()->params_.size(), 0.0);
	j.resize(scurve.begin()->params_.size(), 0.0);

	for (int i = 0; i < scurve.begin()->params_.size(); ++i) {
		last_last_last_p[i] = scurve.begin()->params_[i].pa_;
		last_last_p[i] = scurve.begin()->params_[i].pa_;
		last_p[i] = scurve.begin()->params_[i].pa_;
		p[i] = scurve.begin()->params_[i].pa_;
	}

	const double dt = 0.01;
	LargeNum t = scurve.begin()->params_[0].t0_;
	auto iter = scurve.begin();

	while (iter != scurve.end()) {
		// 测试间隔为 dt 的所有点 //
		while (iter->params_[0].t0_ + iter->params_[0].T_ > t) {
			std::swap(last_last_last_p, last_last_p);
			std::swap(last_last_p, last_p);
			std::swap(last_p, p);
			for (int i = 0; i < scurve.begin()->params_.size(); ++i) {
				aris::plan::s_scurve_at(iter->params_[i], t, &p[i], &v[i], &a[i], &j[i]);

				double max_v = iter->params_[i].vc_max_;
				double max_a = iter->params_[i].a_;
				double max_j = iter->params_[i].j_;

				for (auto it = iter; it != scurve.begin(); it = std::prev(it)) {
					if (t - it->params_[0].t0_ < 4 * dt) {
						max_v = std::max(std::prev(it)->params_[i].vc_max_, max_v);
						max_a = std::max(std::prev(it)->params_[i].a_, max_a);
						max_j = std::max(std::prev(it)->params_[i].j_, max_j);
					}
					else {
						break;
					}
				}

				
				if (iter->params_[0].T_ - t < 4 * dt && std::next(iter) != scurve.end()) {
					max_v = std::max(std::next(iter)->params_[i].vc_max_, max_v);
					max_a = std::max(std::next(iter)->params_[i].a_, max_a);
					max_j = std::max(std::next(iter)->params_[i].j_, max_j);
				}

				double p_current = p[i];
				double p_last = last_p[i];
				double p_last_last = last_last_p[i];
				double p_last_last_last = last_last_last_p[i];

				double v_current = (p_current - p_last) / dt;
				double v_last = (p_last - p_last_last) / dt;
				double v_last_last = (p_last_last - p_last_last_last) / dt;

				double a_current = (v_current - v_last) / dt;
				double a_last = (v_last - v_last_last) / dt;

				double j_current = (a_current - a_last) / dt;

				if (v[i] > max_v + 1e-8 || std::abs(v[i] - v_current) > max_a * dt + 1e-8 || std::abs(v_current) > max_v + 1e-8 * max_v) {
					//auto pr = std::prev(iter);
					aris::plan::s_scurve_at(iter->params_[i], t - 0.01, &last_p[i], &v[i], &a[i], &j[i]);
					aris::plan::s_scurve_at(iter->params_[i], t, &p[i], &v[i], &a[i], &j[i]);
					THROW_FILE_LINE("check velocity failed");
				}
				if (a[i] > max_a + 1e-6 || std::abs(a[i] - a_current) > max_j * dt + 1e-6 || std::abs(a_current) > max_a + 1e-6 * max_a) {
					auto prev_iter = std::prev(iter);
					auto pprev_iter = std::prev(prev_iter);
					
					auto param2 = iter->params_[i];
					std::cout << std::setprecision(15) << param2.Ta_ * (param2.va_ + param2.vc_) / 2 + param2.Tb_ * (param2.vb_ + param2.vc_) / 2 + (param2.T_ - param2.Ta_ - param2.Tb_) * param2.vc_ << std::endl;
					std::cout << param2.pb_ - param2.pa_ << std::endl;

					auto s_acc_time = [](double va, double vb, double a, double j)noexcept->double {
						double v_diff = std::abs(vb - va);
						return v_diff > a * a / j ? v_diff / a + a / j : 2 * std::sqrt(v_diff / j);
					};

					std::cout << s_acc_time(param2.va_, param2.vc_, param2.a_, param2.j_);

					aris::plan::s_scurve_at(iter->params_[i], t, &p[i], &v[i], &a[i], &j[i]);
					THROW_FILE_LINE("check acceleration failed");
				}
				if (j[i] > max_j + 1e-3 || std::abs(j_current) > max_j + 1e-3 * max_j) {
					aris::plan::s_scurve_at(iter->params_[i], t, &p[i], &v[i], &a[i], &j[i]);
					THROW_FILE_LINE("check jerk failed");
				}
			}
			t = t + dt;
		}

		// 测试结束点 是否满足 vb //
		for (int i = 0; i < scurve.begin()->params_.size(); ++i) {
			LargeNum current_p, last_p;
			aris::plan::s_scurve_at(iter->params_[i], iter->params_[i].t0_ + iter->params_[i].T_, &current_p, &v[i], &a[i], &j[i]);
			aris::plan::s_scurve_at(iter->params_[i], std::max(iter->params_[i].t0_, iter->params_[i].t0_ + iter->params_[i].T_ - dt), &last_p, &v[i], &a[i], &j[i]);

			if (std::abs(current_p - last_p) / dt > iter->params_[i].vb_ + iter->params_[i].a_ * dt) {
				THROW_FILE_LINE("check velocity end failed");
			}
		}
		iter++;
	}
}

void test_multi_s_curve(){
	constexpr int m = 1000, n = 5;
	const double pb_v = 10;
	double vc_max_v = 100;
	double vb_max_v = 100;
	double a_v = 1000;
	double j_v = 1000;

	std::vector<double> pb(m * n), vb_max(m * n), vc_max(m * n), a(m * n), j(m * n);
	
	for (int i = 0; i < m * n; ++i) {
		double value;
		value = drand() * pb_v;
		pb[i] = value < 2 ? 0.0 : value - 2;
		value = drand() * vc_max_v;
		vc_max[i] = std::max(value, 0.01);
		value = drand() * vb_max_v;
		vb_max[i] = value < 1 ? 0.0 : value - 1;
		value = drand() * a_v;
		a[i] = std::max(value, 0.01);
		value = drand() * j_v;
		j[i] = std::max(value, 0.01);
	}
	
	for (int i = 1; i < m; ++i) {
		for (int k = 0; k < n; ++k) {
			pb[aris::dynamic::at(i, k, n)] += pb[aris::dynamic::at(i - 1, k, n)];
		}
	}

//#define DEBUG_ERROR

#ifdef DEBUG_ERROR
	auto data = aris::dynamic::dlmread("C:\\Users\\py033\\Desktop\\test_data\\pb.txt");
	aris::dynamic::s_vc(m * n, data.data(), pb.data());
	data = aris::dynamic::dlmread("C:\\Users\\py033\\Desktop\\test_data\\vb_max.txt");
	aris::dynamic::s_vc(m * n, data.data(), vb_max.data());
	data = aris::dynamic::dlmread("C:\\Users\\py033\\Desktop\\test_data\\vc_max.txt");
	aris::dynamic::s_vc(m * n, data.data(), vc_max.data());
	data = aris::dynamic::dlmread("C:\\Users\\py033\\Desktop\\test_data\\a.txt");
	aris::dynamic::s_vc(m * n, data.data(), a.data());
	data = aris::dynamic::dlmread("C:\\Users\\py033\\Desktop\\test_data\\j.txt");
	aris::dynamic::s_vc(m * n, data.data(), j.data());

	vc_max_v = *std::max_element(vc_max.begin(), vc_max.end());
	vb_max_v = *std::max_element(vb_max.begin(), vb_max.end());
	a_v = *std::max_element(a.begin(), a.end());
	j_v = *std::max_element(j.begin(), j.end());
#else

	aris::dynamic::dlmwrite(m, n, pb.data(), "C:\\Users\\py033\\Desktop\\test_data\\pb.txt");
	aris::dynamic::dlmwrite(m, n, vb_max.data(), "C:\\Users\\py033\\Desktop\\test_data\\vb_max.txt");
	aris::dynamic::dlmwrite(m, n, vc_max.data(), "C:\\Users\\py033\\Desktop\\test_data\\vc_max.txt");
	aris::dynamic::dlmwrite(m, n, a.data(), "C:\\Users\\py033\\Desktop\\test_data\\a.txt");
	aris::dynamic::dlmwrite(m, n, j.data(), "C:\\Users\\py033\\Desktop\\test_data\\j.txt");



#endif
	
	//aris::dynamic::dsp(m, n, pb);
	//aris::dynamic::dsp(m, n, vb_max);
	//aris::dynamic::dsp(m, n, vc_max);
	//aris::dynamic::dsp(m, n, a);
	//aris::dynamic::dsp(m, n, j);

	std::list<SCurveNode> scurve;
	for (int i = 0; i < m; ++i) {
		scurve.push_back(SCurveNode{});
		for (int k = 0; k < n; ++k) {
			scurve.back().params_.push_back(SCurveParam{});
			auto &vec = scurve.back().params_.back();

			vec.pb_ = pb[aris::dynamic::at(i, k, n)];
			vec.vb_max_ = vb_max[aris::dynamic::at(i, k, n)];
			vec.vc_max_ = vc_max[aris::dynamic::at(i, k, n)];
			vec.a_ = a[aris::dynamic::at(i, k, n)];
			vec.j_ = j[aris::dynamic::at(i, k, n)];

			if (i == 0)
				vec.pa_ = 0.0;
		}
	}

	try {
		s_compute_scurve(scurve.begin(), scurve.end());
		test_s_curve_vaj(scurve);
	}
	catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		std::exit(-1);
	}
}

void test_scurve(){
	
	aris::plan::SCurveParam p;
	p.T_ = 0.012630497258906562;
	p.a_ = 2.5;
	p.j_ = 10.0;
	p.pa_ = 0.0010698079978823678;
	p.pb_ = 0.0013024434732263595;
	p.t0_ = 4.721714800746539;
	p.va_ = 0.0;
	p.vb_max_ = 0.005037742931539781;
	p.vc_max_ = 3.141592653589793;

	//aris::plan::s_compute_scurve_node(p);
	//auto [Tmax, Tmin] = s_compute_scurve_Tmax_Tmin(p, 0.001);
	//
	
	
	
	
	
	
	
	
	
	std::cout << std::endl << "-----------------test scurve---------------------" << std::endl;
	for (int i=0;;++i) {
		std::cout << i << std::endl;
		test_multi_s_curve();
	}
	
	std::cout << "-----------------test scurve finished------------" << std::endl << std::endl;
}

