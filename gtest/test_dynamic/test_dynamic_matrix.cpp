#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <aris/dynamic/dynamic.hpp>

namespace {

constexpr double kTol = 1e-10;
constexpr double kGuard = -7.65432123456789e210;

auto expect_matrix_near(const double *a, const double *b, int m, int n, double tol = kTol) -> void {
	for (int i = 0; i < m * n; ++i) {
		EXPECT_NEAR(a[i], b[i], tol) << "mismatch at index " << i;
	}
}

auto make_identity(int n) -> std::vector<double> {
	std::vector<double> eye(static_cast<std::size_t>(n * n), 0.0);
	aris::dynamic::s_eye(n, eye.data());
	return eye;
}

auto expect_orthogonal(const double *q, int n, double tol = 1e-8) -> void {
	std::vector<double> qqt(static_cast<std::size_t>(n * n), 0.0);
	const auto eye = make_identity(n);
	aris::dynamic::s_mm(n, n, n, q, aris::dynamic::T(n), q, n, qqt.data(), n);
	expect_matrix_near(qqt.data(), eye.data(), n, n, tol);
}

auto expect_hessenberg(const double *h, int n, double tol = 1e-10) -> void {
	for (int i = 2; i < n; ++i) {
		for (int j = 0; j < i - 1; ++j) {
			EXPECT_NEAR(h[i * n + j], 0.0, tol) << "hessenberg violation at (" << i << "," << j << ")";
		}
	}
}

auto make_legacy_pool() -> std::vector<double> {
	// Original value block used by legacy matrix tests (test_svd/test_hess/test_schur/test_eigen).
	const std::array<double, 72> seed{
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127,
	};

	std::vector<double> data(25 * 24, 0.0);
	for (std::size_t i = 0; i < data.size(); ++i) {
		data[i] = seed[i % seed.size()];
	}
	return data;
}

auto copy_with_stride(int m, int n, const double *src, int src_ld) -> std::vector<double> {
	std::vector<double> out(static_cast<std::size_t>(m * n), 0.0);
	aris::dynamic::s_mc(m, n, src, src_ld, out.data(), n);
	return out;
}

template<typename AType>
auto copy_with_type(int m, int n, const double *src, AType a_t) -> std::vector<double> {
	std::vector<double> out(static_cast<std::size_t>(m * n), 0.0);
	aris::dynamic::s_mc(m, n, src, a_t, out.data(), n);
	return out;
}

template<typename Type>
auto build_type_mask(int rows, int cols, Type t, std::size_t buf_size) -> std::vector<char> {
	std::vector<double> probe(buf_size, std::numeric_limits<double>::quiet_NaN());
	std::vector<double> src(static_cast<std::size_t>(rows * cols), 0.0);
	for (std::size_t i = 0; i < src.size(); ++i) src[i] = 1.0 + static_cast<double>(i) * 1e-3;
	aris::dynamic::s_mc(rows, cols, src.data(), cols, probe.data(), t);
	std::vector<char> mask(probe.size(), 0);
	for (std::size_t i = 0; i < probe.size(); ++i) mask[i] = !std::isnan(probe[i]);
	return mask;
}

auto build_row_major_mask(int rows, int cols, int ld) -> std::vector<char> {
	return build_type_mask(rows, cols, ld, static_cast<std::size_t>(rows * ld));
}

auto expect_guard_unchanged(const std::vector<double> &buf, const std::vector<char> &mask, const std::string &tag) -> void {
	SCOPED_TRACE(tag);
	ASSERT_EQ(buf.size(), mask.size());
	for (std::size_t i = 0; i < buf.size(); ++i) {
		if (!mask[i]) EXPECT_DOUBLE_EQ(buf[i], kGuard) << "unexpected write at index " << i;
	}
}

auto expect_full_unchanged(const std::vector<double> &before, const std::vector<double> &after, const std::string &tag) -> void {
	SCOPED_TRACE(tag);
	ASSERT_EQ(before.size(), after.size());
	for (std::size_t i = 0; i < before.size(); ++i) {
		EXPECT_DOUBLE_EQ(after[i], before[i]) << "unexpected input write at index " << i;
	}
}

template<typename AType>
auto run_hessenberg_case(int m, const double *a, AType a_t, const std::string &tag) -> void {
	SCOPED_TRACE(tag);
	const int h_t = m + 2;
	const int u_t = m + 3;
	std::vector<double> h(static_cast<std::size_t>(m * h_t), kGuard);
	std::vector<double> u(static_cast<std::size_t>(m * u_t), kGuard);
	std::vector<double> uh(static_cast<std::size_t>(m * m), 0.0);
	std::vector<double> reconstructed(static_cast<std::size_t>(m * m), 0.0);
	auto a_dense = copy_with_type(m, m, a, a_t);
	auto h_mask = build_row_major_mask(m, m, h_t);
	auto u_mask = build_row_major_mask(m, m, u_t);

	aris::dynamic::s_hessenberg(m, a, a_t, h.data(), h_t, u.data(), u_t);
	expect_guard_unchanged(h, h_mask, tag + "-H");
	expect_guard_unchanged(u, u_mask, tag + "-U");
	auto h_dense = copy_with_type(m, m, h.data(), h_t);
	auto u_dense = copy_with_type(m, m, u.data(), u_t);
	aris::dynamic::s_mm(m, m, m, u_dense.data(), h_dense.data(), uh.data());
	aris::dynamic::s_mm(m, m, m, uh.data(), m, u_dense.data(), aris::dynamic::T(m), reconstructed.data(), m);

	expect_hessenberg(h_dense.data(), m, 1e-8);
	expect_orthogonal(u_dense.data(), m, 1e-8);
	expect_matrix_near(reconstructed.data(), a_dense.data(), m, m, 1e-7);
}

template<typename AType>
auto run_schur_case(int m, const double *a, AType a_t, const std::string &tag) -> void {
	SCOPED_TRACE(tag);
	const int h_t = m + 1;
	const int t_t = m + 2;
	const int u_t = m + 3;
	std::vector<double> h(static_cast<std::size_t>(m * h_t), kGuard);
	std::vector<double> t(static_cast<std::size_t>(m * t_t), kGuard);
	std::vector<double> u(static_cast<std::size_t>(m * u_t), kGuard);
	std::vector<double> ut(static_cast<std::size_t>(m * m), 0.0);
	std::vector<double> reconstructed(static_cast<std::size_t>(m * m), 0.0);
	auto h_mask = build_row_major_mask(m, m, h_t);
	auto t_mask = build_row_major_mask(m, m, t_t);
	auto u_mask = build_row_major_mask(m, m, u_t);

	aris::dynamic::s_hessenberg(m, a, a_t, h.data(), h_t, nullptr, 1);
	auto h_before = h;
	std::vector<double> eye(static_cast<std::size_t>(m * m), 0.0);
	aris::dynamic::s_eye(m, eye.data());
	aris::dynamic::s_mc(m, m, eye.data(), m, u.data(), u_t);
	const int ret = aris::dynamic::s_schur(m, h.data(), h_t, t.data(), t_t, u.data(), u_t);
	EXPECT_GE(ret, 0);
	expect_matrix_near(h.data(), h_before.data(), 1, static_cast<int>(h.size()), 0.0);
	expect_guard_unchanged(h, h_mask, tag + "-H");
	expect_guard_unchanged(t, t_mask, tag + "-T");
	expect_guard_unchanged(u, u_mask, tag + "-U");
	auto h_dense = copy_with_type(m, m, h.data(), h_t);
	auto t_dense = copy_with_type(m, m, t.data(), t_t);
	auto u_dense = copy_with_type(m, m, u.data(), u_t);

	aris::dynamic::s_mm(m, m, m, u_dense.data(), t_dense.data(), ut.data());
	aris::dynamic::s_mm(m, m, m, ut.data(), m, u_dense.data(), aris::dynamic::T(m), reconstructed.data(), m);

	expect_hessenberg(t_dense.data(), m, 1e-8);
	expect_orthogonal(u_dense.data(), m, 1e-8);
	expect_matrix_near(reconstructed.data(), h_dense.data(), m, m, 1e-6);
}

template<typename AType>
auto run_eigen_case(int m, const double *a, AType a_t, const std::string &tag) -> void {
	SCOPED_TRACE(tag);
	const int e_t = m + 2;
	const int u_t = m + 3;
	std::vector<double> e(static_cast<std::size_t>(m * e_t), kGuard);
	std::vector<double> u(static_cast<std::size_t>(m * u_t), kGuard);
	std::vector<double> ue(static_cast<std::size_t>(m * m), 0.0);
	std::vector<double> reconstructed(static_cast<std::size_t>(m * m), 0.0);
	auto a_dense = copy_with_type(m, m, a, a_t);
	auto e_mask = build_row_major_mask(m, m, e_t);
	auto u_mask = build_row_major_mask(m, m, u_t);

	const int ret = aris::dynamic::s_eigen(m, a, a_t, e.data(), e_t, u.data(), u_t);
	EXPECT_GE(ret, 0);
	expect_guard_unchanged(e, e_mask, tag + "-E");
	expect_guard_unchanged(u, u_mask, tag + "-U");
	auto e_dense = copy_with_type(m, m, e.data(), e_t);
	auto u_dense = copy_with_type(m, m, u.data(), u_t);

	aris::dynamic::s_mm(m, m, m, u_dense.data(), e_dense.data(), ue.data());
	aris::dynamic::s_mm(m, m, m, ue.data(), m, u_dense.data(), aris::dynamic::T(m), reconstructed.data(), m);

	expect_hessenberg(e_dense.data(), m, 1e-8);
	expect_orthogonal(u_dense.data(), m, 1e-8);
	expect_matrix_near(reconstructed.data(), a_dense.data(), m, m, 1e-6);
}

template<typename AType>
auto run_svd_case(int m, int n, const double *a, AType a_t, const std::string &tag) -> void {
	SCOPED_TRACE(tag);
	const int u_t = m + 2;
	const int s_t = n + 3;
	const int v_t = n + 2;
	std::vector<double> u(static_cast<std::size_t>(m * u_t), kGuard);
	std::vector<double> s(static_cast<std::size_t>(m * s_t), kGuard);
	std::vector<double> v(static_cast<std::size_t>(n * v_t), kGuard);
	std::vector<double> us(static_cast<std::size_t>(m * n), 0.0);
	std::vector<double> reconstructed(static_cast<std::size_t>(m * n), 0.0);
	auto a_dense = copy_with_type(m, n, a, a_t);

	auto u_mask = build_row_major_mask(m, m, u_t);
	auto s_mask = build_row_major_mask(m, n, s_t);
	auto v_mask = build_row_major_mask(n, n, v_t);

	aris::dynamic::s_svd(m, n, a, a_t, u.data(), u_t, s.data(), s_t, v.data(), v_t);
	expect_guard_unchanged(u, u_mask, tag + "-U");
	expect_guard_unchanged(s, s_mask, tag + "-S");
	expect_guard_unchanged(v, v_mask, tag + "-V");

	auto u_dense = copy_with_type(m, m, u.data(), u_t);
	auto s_dense = copy_with_type(m, n, s.data(), s_t);
	auto v_dense = copy_with_type(n, n, v.data(), v_t);
	aris::dynamic::s_mm(m, n, m, u_dense.data(), m, s_dense.data(), n, us.data(), n);
	aris::dynamic::s_mm(m, n, n, us.data(), n, v_dense.data(), aris::dynamic::T(n), reconstructed.data(), n);

	expect_matrix_near(reconstructed.data(), a_dense.data(), m, n, 1e-7);
	expect_orthogonal(u_dense.data(), m, 1e-7);
	expect_orthogonal(v_dense.data(), n, 1e-7);

	for (int i = 1; i < std::min(m, n); ++i) {
		EXPECT_LE(s_dense[i * n + i], s_dense[(i - 1) * n + (i - 1)] + 1e-10);
	}
}

TEST(DynamicMatrixTest, BasicOperation) {
	const double x[3]{1.0, 2.0, 3.0};
	const double y[3]{4.0, 5.0, 6.0};
	constexpr int x_t = 2;
	constexpr int y_t = 3;
	constexpr int z_t = 4;
	std::vector<double> x_buf(3 * x_t, kGuard);
	std::vector<double> y_buf(3 * y_t, kGuard);
	std::vector<double> z_buf(3 * z_t, kGuard);
	auto x_mask = build_row_major_mask(3, 1, x_t);
	auto y_mask = build_row_major_mask(3, 1, y_t);
	auto z_mask = build_row_major_mask(3, 1, z_t);
	aris::dynamic::s_vc(3, x, 1, x_buf.data(), x_t);
	aris::dynamic::s_vc(3, y, 1, y_buf.data(), y_t);
	auto x_before = x_buf;
	auto y_before = y_buf;

	EXPECT_NEAR(aris::dynamic::s_vv(3, x, y), 32.0, kTol);

	aris::dynamic::s_vc(3, y_buf.data(), y_t, z_buf.data(), z_t);
	expect_full_unchanged(y_before, y_buf, "basic-vc-input");
	expect_guard_unchanged(z_buf, z_mask, "basic-vc-output");
	aris::dynamic::s_va(3, x_buf.data(), x_t, z_buf.data(), z_t);
	expect_full_unchanged(x_before, x_buf, "basic-va-input");
	expect_guard_unchanged(z_buf, z_mask, "basic-va-output");
	auto z_dense = copy_with_type(3, 1, z_buf.data(), z_t);
	const double expected_sum[3]{5.0, 7.0, 9.0};
	expect_matrix_near(z_dense.data(), expected_sum, 1, 3);

	aris::dynamic::s_vs(3, x_buf.data(), x_t, z_buf.data(), z_t);
	expect_full_unchanged(x_before, x_buf, "basic-vs-input");
	expect_guard_unchanged(z_buf, z_mask, "basic-vs-output");
	z_dense = copy_with_type(3, 1, z_buf.data(), z_t);
	expect_matrix_near(z_dense.data(), y, 1, 3);

	aris::dynamic::s_nv(3, 0.5, z_buf.data(), z_t);
	expect_guard_unchanged(z_buf, z_mask, "basic-nv-output");
	z_dense = copy_with_type(3, 1, z_buf.data(), z_t);
	const double expected_scaled[3]{2.0, 2.5, 3.0};
	expect_matrix_near(z_dense.data(), expected_scaled, 1, 3);
	expect_guard_unchanged(x_buf, x_mask, "basic-x-guard");
	expect_guard_unchanged(y_buf, y_mask, "basic-y-guard");
}

TEST(DynamicMatrixTest, SpecificMatrix) {
	const double a[6]{1.0, 2.0, 3.0,
		4.0, 5.0, 6.0};
	std::vector<double> col_major(4 * 3, kGuard);
	std::vector<double> round_trip(2 * 5, kGuard);
	auto col_mask = build_type_mask(2, 3, aris::dynamic::ColMajor{4}, col_major.size());
	auto round_mask = build_row_major_mask(2, 3, 5);

	aris::dynamic::s_mc(2, 3, a, 3, col_major.data(), aris::dynamic::ColMajor{4});
	expect_guard_unchanged(col_major, col_mask, "specific-col-major");
	aris::dynamic::s_mc(2, 3, col_major.data(), aris::dynamic::ColMajor{4}, round_trip.data(), 5);
	expect_guard_unchanged(round_trip, round_mask, "specific-round-trip");
	auto round_dense = copy_with_type(2, 3, round_trip.data(), 5);
	expect_matrix_near(round_dense.data(), a, 2, 3);

	const auto eye = make_identity(3);
	std::vector<double> a_pad(2 * 6, kGuard);
	std::vector<double> right(2 * 7, kGuard);
	auto a_mask = build_row_major_mask(2, 3, 6);
	auto right_mask = build_row_major_mask(2, 3, 7);
	aris::dynamic::s_mc(2, 3, a, 3, a_pad.data(), 6);
	auto a_before = a_pad;
	aris::dynamic::s_mm(2, 3, 3, a_pad.data(), 6, eye.data(), 3, right.data(), 7);
	expect_full_unchanged(a_before, a_pad, "specific-mm-input");
	expect_guard_unchanged(a_pad, a_mask, "specific-a-guard");
	expect_guard_unchanged(right, right_mask, "specific-right-guard");
	auto right_dense = copy_with_type(2, 3, right.data(), 7);
	expect_matrix_near(right_dense.data(), a, 2, 3);
}

TEST(DynamicMatrixTest, Multiply) {
	const double a[6]{1.0, 2.0, 3.0,
		4.0, 5.0, 6.0};
	const double b[6]{7.0, 8.0,
		9.0, 10.0,
		11.0, 12.0};
	std::vector<double> a_pad(2 * 5, kGuard);
	std::vector<double> b_pad(3 * 4, kGuard);
	std::vector<double> c_pad(2 * 6, kGuard);
	auto a_mask = build_row_major_mask(2, 3, 5);
	auto b_mask = build_row_major_mask(3, 2, 4);
	auto c_mask = build_row_major_mask(2, 2, 6);
	aris::dynamic::s_mc(2, 3, a, 3, a_pad.data(), 5);
	aris::dynamic::s_mc(3, 2, b, 2, b_pad.data(), 4);
	auto a_before = a_pad;
	auto b_before = b_pad;

	aris::dynamic::s_mm(2, 2, 3, a_pad.data(), 5, b_pad.data(), 4, c_pad.data(), 6);
	expect_full_unchanged(a_before, a_pad, "multiply-a-input");
	expect_full_unchanged(b_before, b_pad, "multiply-b-input");
	expect_guard_unchanged(a_pad, a_mask, "multiply-a-guard");
	expect_guard_unchanged(b_pad, b_mask, "multiply-b-guard");
	expect_guard_unchanged(c_pad, c_mask, "multiply-c-guard");
	auto c_dense = copy_with_type(2, 2, c_pad.data(), 6);

	const double expected[4]{58.0, 64.0, 139.0, 154.0};
	expect_matrix_near(c_dense.data(), expected, 2, 2);
}

TEST(DynamicMatrixTest, LegacyMultiplyOperationScenarios) {
	const double a[6]{0.498364051982143,0.959743958516081,0.340385726666133,
		0.585267750979777,0.223811939491137,0.751267059305653};
	const double b[12]{0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,
		0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,
		0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616};

	std::vector<double> c_expected(8, 0.0);
	aris::dynamic::s_mm(2, 4, 3, a, b, c_expected.data());

	std::vector<double> a_rm(2 * 6, 0.0), b_rm(3 * 7, 0.0), c_rm(2 * 5, 0.0);
	aris::dynamic::s_mc(2, 3, a, 3, a_rm.data(), 6);
	aris::dynamic::s_mc(3, 4, b, 4, b_rm.data(), 7);

	aris::dynamic::s_mm(2, 4, 3, a_rm.data(), 6, b_rm.data(), 7, c_rm.data(), 5);
	auto c_dense = copy_with_type(2, 4, c_rm.data(), 5);
	expect_matrix_near(c_dense.data(), c_expected.data(), 2, 4, 1e-10);

	std::vector<double> a_cm(5 * 3, 0.0), b_cm(6 * 4, 0.0), c1(8, 0.0), c2(8, 0.0), c3(8, 0.0);
	aris::dynamic::s_mc(2, 3, a, 3, a_cm.data(), aris::dynamic::ColMajor{5});
	aris::dynamic::s_mc(3, 4, b, 4, b_cm.data(), aris::dynamic::ColMajor{6});

	aris::dynamic::s_mm(2, 4, 3,
		a_cm.data(), aris::dynamic::ColMajor{5},
		b_cm.data(), aris::dynamic::ColMajor{6},
		c1.data(), 4);
	expect_matrix_near(c1.data(), c_expected.data(), 2, 4, 1e-10);

	std::fill(c2.begin(), c2.end(), 1.0);
	aris::dynamic::s_mma(2, 4, 3, a, b, c2.data());
	for (std::size_t i = 0; i < c2.size(); ++i) EXPECT_NEAR(c2[i], c_expected[i] + 1.0, 1e-10);

	std::fill(c3.begin(), c3.end(), 1.0);
	aris::dynamic::s_mms(2, 4, 3, a, b, c3.data());
	for (std::size_t i = 0; i < c3.size(); ++i) EXPECT_NEAR(c3[i], 1.0 - c_expected[i], 1e-10);

	std::vector<double> c4(8, 0.0);
	aris::dynamic::s_mmi(2, 4, 3, a, b, c4.data());
	for (std::size_t i = 0; i < c4.size(); ++i) EXPECT_NEAR(c4[i], -c_expected[i], 1e-10);

	std::vector<double> p_src{0.3500,0.1966,0.2511,0.6160,0.4733,0.3517,0.8308,0.5853,0.5497,0.9172};
	std::vector<double> p_expected{0.8308,0.3517,0.3500,0.6160,0.5853,0.4733,0.2511,0.1966,0.5497,0.9172};
	aris::Size p[10]{6,5,0,3,7,4,2,1,8,9};
	auto p_work = p_src;
	aris::dynamic::s_permutate(10, 1, p, p_work.data(), 1);
	expect_matrix_near(p_work.data(), p_expected.data(), 1, 10, 1e-10);
	aris::dynamic::s_permutate_inv(10, 1, p, p_work.data(), 1);
	expect_matrix_near(p_work.data(), p_src.data(), 1, 10, 1e-10);
}

TEST(DynamicMatrixTest, Llt) {
	const double a[36]{
		1.82553083943141,1.42060601118548,1.36736238745112,1.50658906468564,1.86464891726001,1.04079482779702,
		1.42060601118548,2.10941693872417,1.92463386848915,1.23889223270807,2.23186828169132,1.22211204078486,
		1.36736238745112,1.92463386848915,2.06653199450749,1.37659815598197,2.07988145626914,1.30113287432829,
		1.50658906468564,1.23889223270807,1.37659815598197,1.69212820994619,1.67619205287543,0.914095057763804,
		1.86464891726001,2.23186828169132,2.07988145626914,1.67619205287543,3.0881251584706,1.69495025317372,
		1.04079482779702,1.22211204078486,1.30113287432829,0.914095057763804,1.69495025317372,1.17872570447206,
	};
	const double b[12]{
		0.765516788149002,0.709364830858073,
		0.795199901137063,0.754686681982361,
		0.186872604554379,0.276025076998578,
		0.489764395788231,0.679702676853675,
		0.445586200710899,0.655098003973841,
		0.646313010111265,0.162611735194631,
	};

	constexpr int l_t = 8;
	std::vector<double> l_buf(6 * l_t, kGuard);
	auto l_mask = build_row_major_mask(6, 6, l_t);
	double l_lower[36]{};
	double reconstructed[36]{};
	double x[12]{};
	double check_b[12]{};

	aris::dynamic::s_llt(6, a, 6, l_buf.data(), l_t);
	expect_guard_unchanged(l_buf, l_mask, "llt-base-L");
	auto l_dense = copy_with_type(6, 6, l_buf.data(), l_t);
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			l_lower[i * 6 + j] = (j <= i) ? l_dense[i * 6 + j] : 0.0;
		}
	}
	aris::dynamic::s_mm(6, 6, 6, l_lower, 6, l_lower, aris::dynamic::T(6), reconstructed, 6);
	expect_matrix_near(reconstructed, a, 6, 6, 1e-8);

	// Legacy test also solved linear systems on the same matrix.
	aris::dynamic::s_sov_lm(6, 2, l_lower, b, x);
	aris::dynamic::s_mm(6, 2, 6, l_lower, x, check_b);
	expect_matrix_near(check_b, b, 6, 2, 1e-8);
}

TEST(DynamicMatrixTest, Householder) {
	const double a[12]{
		1.0, 2.0, 3.0,
		0.0, 1.0, 4.0,
		5.0, 6.0, 0.0,
		2.0, 1.0, 1.0};
	constexpr int u_t = 5;
	constexpr int tau_t = 2;
	std::vector<double> u_buf(4 * u_t, kGuard);
	std::vector<double> tau_buf(3 * tau_t, kGuard);
	auto u_mask = build_row_major_mask(4, 3, u_t);
	auto tau_mask = build_row_major_mask(3, 1, tau_t);
	double q[16]{};
	double r[12]{};
	double reconstructed[12]{};

	aris::dynamic::s_householder_ut(4, 3, a, 3, u_buf.data(), u_t, tau_buf.data(), tau_t);
	expect_guard_unchanged(u_buf, u_mask, "householder-base-U");
	expect_guard_unchanged(tau_buf, tau_mask, "householder-base-tau");
	aris::dynamic::s_householder_ut2qr(4, 3, u_buf.data(), u_t, tau_buf.data(), tau_t, q, 4, r, 3);
	aris::dynamic::s_mm(4, 3, 4, q, 4, r, 3, reconstructed, 3);

	expect_matrix_near(reconstructed, a, 4, 3, 1e-8);
	expect_orthogonal(q, 4, 1e-8);

	const double x_true[3]{0.3, -1.2, 2.1};
	double b[4]{};
	double x_solved[4]{};
	aris::dynamic::s_mm(4, 1, 3, a, x_true, b);
	aris::dynamic::s_householder_ut_sov(4, 3, 1, u_buf.data(), u_t, tau_buf.data(), tau_t, b, 1, x_solved, 1);
	expect_matrix_near(x_solved, x_true, 1, 3, 1e-8);
}

TEST(DynamicMatrixTest, Hess) {
	const auto pool = make_legacy_pool();
	const std::array<int, 10> dims{0, 1, 2, 3, 5, 10, 15, 16, 23, 24};

	for (const int m : dims) {
		if (m == 0) {
			continue;
		}
		run_hessenberg_case(m, pool.data(), 24, "row-major");
	}
}

TEST(DynamicMatrixTest, Schur) {
	const auto pool = make_legacy_pool();
	const std::array<int, 9> dims{1, 2, 3, 5, 10, 15, 16, 23, 24};

	for (const int m : dims) {
		run_schur_case(m, pool.data(), 24, "row-major");
	}
}

TEST(DynamicMatrixTest, Eigen) {
	const auto pool = make_legacy_pool();
	const std::array<int, 9> dims{1, 2, 3, 5, 10, 15, 16, 23, 24};

	for (const int m : dims) {
		run_eigen_case(m, pool.data(), 24, "row-major");
	}
}

TEST(DynamicMatrixTest, Svd) {
	const auto pool = make_legacy_pool();
	std::vector<double> neg_pool = pool;
	aris::dynamic::s_nm(25, 24, -1.0, neg_pool.data(), 24);

	std::vector<double> zeros(25 * 24, 0.0);
	std::vector<double> eye24(25 * 24, 0.0);
	aris::dynamic::s_eye(24, eye24.data(), 24);

	const std::vector<std::pair<int, int>> dims{
		{1, 1}, {2, 1}, {3, 1}, {5, 1}, {10, 1},
		{1, 2}, {1, 3}, {1, 5}, {1, 10},
		{2, 2}, {3, 2}, {4, 2}, {7, 2}, {10, 2},
		{2, 3}, {2, 4}, {2, 7}, {2, 10},
		{3, 3}, {4, 3}, {5, 3}, {7, 3}, {10, 3},
		{3, 4}, {3, 5}, {3, 7}, {3, 10},
		{5, 4}, {6, 5}, {7, 6},
		{8, 7}, {10, 9}, {10, 10}, {11, 10}, {10, 11},
		{21, 10}, {10, 21},
		{12, 11}, {13, 12}, {14, 13}, {15, 14}, {16, 15}, {15, 15},
		{20, 15}, {25, 15}, {15, 16}, {17, 16}, {18, 17}, {19, 18}, {20, 19},
		{21, 20}, {22, 21}, {23, 22}, {24, 23}, {25, 24}, {24, 24}, {24, 25}
	};

	for (const auto &[m, n] : dims) {
		run_svd_case(m, n, pool.data(), 24, "pool");
		run_svd_case(m, n, neg_pool.data(), 24, "neg_pool");
	}

	const std::vector<std::pair<int, int>> zero_dims{{2, 1}, {3, 2}, {4, 3}, {5, 4}, {6, 5}, {7, 6}};
	for (const auto &[m, n] : zero_dims) {
		run_svd_case(m, n, zeros.data(), 24, "zeros");
	}

	run_svd_case(2, 1, eye24.data(), 24, "eye24_base");
	run_svd_case(5, 4, eye24.data() + 24, 24, "eye24_row_offset");
	run_svd_case(5, 4, eye24.data() + 1, 24, "eye24_col_offset");
	run_svd_case(4, 3, eye24.data() + 24, 24, "eye24_row_offset_small");
	run_svd_case(4, 3, eye24.data() + 1, 24, "eye24_col_offset_small");
	run_svd_case(4, 3, eye24.data(), 24, "eye24_base_small");
}

TEST(DynamicMatrixTest, MatrixTypeWithMismatchedLeadingDimension) {
	const auto pool = make_legacy_pool();
	constexpr double guard = -7.65432123456789e210;

	auto build_mask_matrix = [&](int m, int n, auto t, std::size_t buf_size) {
		std::vector<double> probe(buf_size, std::numeric_limits<double>::quiet_NaN());
		std::vector<double> src(static_cast<std::size_t>(m * n), 0.0);
		for (std::size_t i = 0; i < src.size(); ++i) src[i] = 1.0 + static_cast<double>(i) * 1e-3;
		aris::dynamic::s_mc(m, n, src.data(), n, probe.data(), t);
		std::vector<char> mask(buf_size, 0);
		for (std::size_t i = 0; i < buf_size; ++i) mask[i] = !std::isnan(probe[i]);
		return mask;
	};

	auto build_mask_vector = [&](int len, auto t, std::size_t buf_size) {
		std::vector<double> probe(buf_size, std::numeric_limits<double>::quiet_NaN());
		std::vector<double> src(static_cast<std::size_t>(len), 0.0);
		for (std::size_t i = 0; i < src.size(); ++i) src[i] = 1.0 + static_cast<double>(i) * 1e-3;
		aris::dynamic::s_mc(len, 1, src.data(), 1, probe.data(), t);
		std::vector<char> mask(buf_size, 0);
		for (std::size_t i = 0; i < buf_size; ++i) mask[i] = !std::isnan(probe[i]);
		return mask;
	};

	auto expect_guard_unchanged = [&](const std::vector<double> &buf, const std::vector<char> &mask, const std::string &tag) {
		SCOPED_TRACE(tag);
		ASSERT_EQ(buf.size(), mask.size());
		for (std::size_t i = 0; i < buf.size(); ++i) {
			if (!mask[i]) {
				EXPECT_DOUBLE_EQ(buf[i], guard) << "unexpected write at index " << i;
			}
		}
	};

	auto expect_full_unchanged = [&](const std::vector<double> &before, const std::vector<double> &after, const std::string &tag) {
		SCOPED_TRACE(tag);
		ASSERT_EQ(before.size(), after.size());
		for (std::size_t i = 0; i < before.size(); ++i) {
			EXPECT_DOUBLE_EQ(after[i], before[i]) << "unexpected input write at index " << i;
		}
	};

	// Rectangular matrix for SVD coverage.
	constexpr int m_rect = 5;
	constexpr int n_rect = 4;
	auto rect_dense = copy_with_type(m_rect, n_rect, pool.data(), 24);

	constexpr int row_ld_rect = n_rect + 5;
	std::vector<double> rect_row_pad(static_cast<std::size_t>(m_rect * row_ld_rect), guard);
	aris::dynamic::s_mc(m_rect, n_rect, rect_dense.data(), n_rect, rect_row_pad.data(), row_ld_rect);

	constexpr int col_ld_rect = m_rect + 3;
	std::vector<double> rect_col_pad(static_cast<std::size_t>(col_ld_rect * n_rect), guard);
	aris::dynamic::s_mc(m_rect, n_rect, rect_dense.data(), n_rect, rect_col_pad.data(), aris::dynamic::ColMajor{col_ld_rect});

	const aris::dynamic::Stride rect_stride{2, 13};
	std::vector<double> rect_stride_pad(64, guard);
	aris::dynamic::s_mc(m_rect, n_rect, rect_dense.data(), n_rect, rect_stride_pad.data(), rect_stride);

	auto run_svd_type_case = [&](const std::vector<double> &a_store, auto a_t, auto u_t, auto s_t, auto v_t, const std::string &tag) {
		SCOPED_TRACE(tag);
		auto a_before = a_store;
		auto a_dense = copy_with_type(m_rect, n_rect, a_store.data(), a_t);

		std::vector<double> u_buf(256, guard);
		std::vector<double> s_buf(256, guard);
		std::vector<double> v_buf(256, guard);
		auto u_mask = build_mask_matrix(m_rect, m_rect, u_t, u_buf.size());
		auto s_mask = build_mask_matrix(m_rect, n_rect, s_t, s_buf.size());
		auto v_mask = build_mask_matrix(n_rect, n_rect, v_t, v_buf.size());

		aris::dynamic::s_svd(m_rect, n_rect, a_store.data(), a_t, u_buf.data(), u_t, s_buf.data(), s_t, v_buf.data(), v_t);

		expect_full_unchanged(a_before, a_store, tag + "-A");
		expect_guard_unchanged(u_buf, u_mask, tag + "-U");
		expect_guard_unchanged(s_buf, s_mask, tag + "-S");
		expect_guard_unchanged(v_buf, v_mask, tag + "-V");

		auto u_dense = copy_with_type(m_rect, m_rect, u_buf.data(), u_t);
		auto s_dense = copy_with_type(m_rect, n_rect, s_buf.data(), s_t);
		auto v_dense = copy_with_type(n_rect, n_rect, v_buf.data(), v_t);

		std::vector<double> us(static_cast<std::size_t>(m_rect * n_rect), 0.0);
		std::vector<double> reconstructed(static_cast<std::size_t>(m_rect * n_rect), 0.0);
		aris::dynamic::s_mm(m_rect, n_rect, m_rect, u_dense.data(), m_rect, s_dense.data(), n_rect, us.data(), n_rect);
		aris::dynamic::s_mm(m_rect, n_rect, n_rect, us.data(), n_rect, v_dense.data(), aris::dynamic::T(n_rect), reconstructed.data(), n_rect);

		expect_matrix_near(reconstructed.data(), a_dense.data(), m_rect, n_rect, 1e-7);
		expect_orthogonal(u_dense.data(), m_rect, 1e-7);
		expect_orthogonal(v_dense.data(), n_rect, 1e-7);
		for (int i = 1; i < std::min(m_rect, n_rect); ++i) {
			EXPECT_LE(s_dense[i * n_rect + i], s_dense[(i - 1) * n_rect + (i - 1)] + 1e-10);
		}
	};

	run_svd_type_case(rect_row_pad, row_ld_rect, 11, 12, 9, "svd-row-major-padded");
	run_svd_type_case(rect_col_pad, aris::dynamic::ColMajor{col_ld_rect}, aris::dynamic::ColMajor{9}, aris::dynamic::ColMajor{8}, aris::dynamic::ColMajor{7}, "svd-col-major-padded");
	run_svd_type_case(rect_stride_pad, rect_stride, aris::dynamic::Stride{2, 17}, aris::dynamic::Stride{3, 19}, aris::dynamic::Stride{2, 13}, "svd-stride-padded");

	// Square matrix for Hess/Schur/Eigen coverage.
	constexpr int m_sq = 6;
	auto sq_dense = copy_with_type(m_sq, m_sq, pool.data(), 24);

	constexpr int row_ld_sq = m_sq + 4;
	std::vector<double> sq_row_pad(static_cast<std::size_t>(m_sq * row_ld_sq), guard);
	aris::dynamic::s_mc(m_sq, m_sq, sq_dense.data(), m_sq, sq_row_pad.data(), row_ld_sq);

	constexpr int col_ld_sq = m_sq + 3;
	std::vector<double> sq_col_pad(static_cast<std::size_t>(col_ld_sq * m_sq), guard);
	aris::dynamic::s_mc(m_sq, m_sq, sq_dense.data(), m_sq, sq_col_pad.data(), aris::dynamic::ColMajor{col_ld_sq});

	const aris::dynamic::Stride sq_stride{2, 17};
	std::vector<double> sq_stride_pad(128, guard);
	aris::dynamic::s_mc(m_sq, m_sq, sq_dense.data(), m_sq, sq_stride_pad.data(), sq_stride);

	auto run_hessenberg_type_case = [&](const std::vector<double> &a_store, auto a_t, auto h_t, auto u_t, const std::string &tag) {
		SCOPED_TRACE(tag);
		auto a_before = a_store;
		auto a_dense = copy_with_type(m_sq, m_sq, a_store.data(), a_t);
		std::vector<double> h_buf(320, guard);
		std::vector<double> u_buf(320, guard);
		auto h_mask = build_mask_matrix(m_sq, m_sq, h_t, h_buf.size());
		auto u_mask = build_mask_matrix(m_sq, m_sq, u_t, u_buf.size());

		aris::dynamic::s_hessenberg(m_sq, a_store.data(), a_t, h_buf.data(), h_t, u_buf.data(), u_t);

		expect_full_unchanged(a_before, a_store, tag + "-A");
		expect_guard_unchanged(h_buf, h_mask, tag + "-H");
		expect_guard_unchanged(u_buf, u_mask, tag + "-U");

		auto h_dense = copy_with_type(m_sq, m_sq, h_buf.data(), h_t);
		auto u_dense = copy_with_type(m_sq, m_sq, u_buf.data(), u_t);
		std::vector<double> uh(static_cast<std::size_t>(m_sq * m_sq), 0.0);
		std::vector<double> reconstructed(static_cast<std::size_t>(m_sq * m_sq), 0.0);
		aris::dynamic::s_mm(m_sq, m_sq, m_sq, u_dense.data(), h_dense.data(), uh.data());
		aris::dynamic::s_mm(m_sq, m_sq, m_sq, uh.data(), m_sq, u_dense.data(), aris::dynamic::T(m_sq), reconstructed.data(), m_sq);

		expect_hessenberg(h_dense.data(), m_sq, 1e-8);
		expect_orthogonal(u_dense.data(), m_sq, 1e-8);
		expect_matrix_near(reconstructed.data(), a_dense.data(), m_sq, m_sq, 1e-7);
	};

	run_hessenberg_type_case(sq_row_pad, row_ld_sq, row_ld_sq + 2, row_ld_sq + 3, "hess-row-major-padded");
	run_hessenberg_type_case(sq_col_pad, aris::dynamic::ColMajor{col_ld_sq}, aris::dynamic::ColMajor{col_ld_sq + 1}, aris::dynamic::ColMajor{col_ld_sq + 2}, "hess-col-major-padded");
	run_hessenberg_type_case(sq_stride_pad, sq_stride, aris::dynamic::Stride{2, 19}, aris::dynamic::Stride{3, 21}, "hess-stride-padded");

	auto run_schur_type_case = [&](const std::vector<double> &a_store, auto a_t, auto h_t, auto t_t, auto u_t, const std::string &tag) {
		SCOPED_TRACE(tag);
		auto a_dense = copy_with_type(m_sq, m_sq, a_store.data(), a_t);
		std::vector<double> h_buf(320, guard);
		std::vector<double> t_buf(320, guard);
		std::vector<double> u_buf(320, guard);
		auto h_mask = build_mask_matrix(m_sq, m_sq, h_t, h_buf.size());
		auto t_mask = build_mask_matrix(m_sq, m_sq, t_t, t_buf.size());
		auto u_mask = build_mask_matrix(m_sq, m_sq, u_t, u_buf.size());

		aris::dynamic::s_hessenberg(m_sq, a_store.data(), a_t, h_buf.data(), h_t, nullptr, 1);
		auto h_before = h_buf;
		auto h_dense = copy_with_type(m_sq, m_sq, h_buf.data(), h_t);

		std::vector<double> eye(static_cast<std::size_t>(m_sq * m_sq), 0.0);
		aris::dynamic::s_eye(m_sq, eye.data());
		aris::dynamic::s_mc(m_sq, m_sq, eye.data(), m_sq, u_buf.data(), u_t);

		const int ret = aris::dynamic::s_schur(m_sq, h_buf.data(), h_t, t_buf.data(), t_t, u_buf.data(), u_t);
		EXPECT_GE(ret, 0);

		expect_full_unchanged(h_before, h_buf, tag + "-H");
		expect_guard_unchanged(h_buf, h_mask, tag + "-H-guard");
		expect_guard_unchanged(t_buf, t_mask, tag + "-T");
		expect_guard_unchanged(u_buf, u_mask, tag + "-U");

		auto t_dense = copy_with_type(m_sq, m_sq, t_buf.data(), t_t);
		auto u_dense = copy_with_type(m_sq, m_sq, u_buf.data(), u_t);
		std::vector<double> ut(static_cast<std::size_t>(m_sq * m_sq), 0.0);
		std::vector<double> reconstructed(static_cast<std::size_t>(m_sq * m_sq), 0.0);
		aris::dynamic::s_mm(m_sq, m_sq, m_sq, u_dense.data(), t_dense.data(), ut.data());
		aris::dynamic::s_mm(m_sq, m_sq, m_sq, ut.data(), m_sq, u_dense.data(), aris::dynamic::T(m_sq), reconstructed.data(), m_sq);

		expect_hessenberg(t_dense.data(), m_sq, 1e-8);
		expect_orthogonal(u_dense.data(), m_sq, 1e-8);
		expect_matrix_near(reconstructed.data(), h_dense.data(), m_sq, m_sq, 1e-6);
		expect_matrix_near(a_dense.data(), a_dense.data(), m_sq, m_sq, 0.0);
	};

	run_schur_type_case(sq_row_pad, row_ld_sq, row_ld_sq + 1, row_ld_sq + 2, row_ld_sq + 3, "schur-row-major-padded");
	run_schur_type_case(sq_col_pad, aris::dynamic::ColMajor{col_ld_sq}, aris::dynamic::ColMajor{col_ld_sq + 1}, aris::dynamic::ColMajor{col_ld_sq + 2}, aris::dynamic::ColMajor{col_ld_sq + 3}, "schur-col-major-padded");
	run_schur_type_case(sq_stride_pad, sq_stride, aris::dynamic::Stride{2, 19}, aris::dynamic::Stride{3, 21}, aris::dynamic::Stride{4, 23}, "schur-stride-padded");

	auto run_eigen_type_case = [&](const std::vector<double> &a_store, auto a_t, auto e_t, auto u_t, const std::string &tag) {
		SCOPED_TRACE(tag);
		auto a_before = a_store;
		auto a_dense = copy_with_type(m_sq, m_sq, a_store.data(), a_t);
		std::vector<double> e_buf(320, guard);
		std::vector<double> u_buf(320, guard);
		auto e_mask = build_mask_matrix(m_sq, m_sq, e_t, e_buf.size());
		auto u_mask = build_mask_matrix(m_sq, m_sq, u_t, u_buf.size());

		const int ret = aris::dynamic::s_eigen(m_sq, a_store.data(), a_t, e_buf.data(), e_t, u_buf.data(), u_t);
		EXPECT_GE(ret, 0);

		expect_full_unchanged(a_before, a_store, tag + "-A");
		expect_guard_unchanged(e_buf, e_mask, tag + "-E");
		expect_guard_unchanged(u_buf, u_mask, tag + "-U");

		auto e_dense = copy_with_type(m_sq, m_sq, e_buf.data(), e_t);
		auto u_dense = copy_with_type(m_sq, m_sq, u_buf.data(), u_t);
		std::vector<double> ue(static_cast<std::size_t>(m_sq * m_sq), 0.0);
		std::vector<double> reconstructed(static_cast<std::size_t>(m_sq * m_sq), 0.0);
		aris::dynamic::s_mm(m_sq, m_sq, m_sq, u_dense.data(), e_dense.data(), ue.data());
		aris::dynamic::s_mm(m_sq, m_sq, m_sq, ue.data(), m_sq, u_dense.data(), aris::dynamic::T(m_sq), reconstructed.data(), m_sq);

		expect_hessenberg(e_dense.data(), m_sq, 1e-8);
		expect_orthogonal(u_dense.data(), m_sq, 1e-8);
		expect_matrix_near(reconstructed.data(), a_dense.data(), m_sq, m_sq, 1e-6);
	};

	run_eigen_type_case(sq_row_pad, row_ld_sq, row_ld_sq + 2, row_ld_sq + 3, "eigen-row-major-padded");
	run_eigen_type_case(sq_col_pad, aris::dynamic::ColMajor{col_ld_sq}, aris::dynamic::ColMajor{col_ld_sq + 1}, aris::dynamic::ColMajor{col_ld_sq + 2}, "eigen-col-major-padded");
	run_eigen_type_case(sq_stride_pad, sq_stride, aris::dynamic::Stride{2, 19}, aris::dynamic::Stride{3, 21}, "eigen-stride-padded");

	// LLT coverage with a_t and output type not matching matrix dimension.
	const double llt_a[36]{
		1.82553083943141,1.42060601118548,1.36736238745112,1.50658906468564,1.86464891726001,1.04079482779702,
		1.42060601118548,2.10941693872417,1.92463386848915,1.23889223270807,2.23186828169132,1.22211204078486,
		1.36736238745112,1.92463386848915,2.06653199450749,1.37659815598197,2.07988145626914,1.30113287432829,
		1.50658906468564,1.23889223270807,1.37659815598197,1.69212820994619,1.67619205287543,0.914095057763804,
		1.86464891726001,2.23186828169132,2.07988145626914,1.67619205287543,3.0881251584706,1.69495025317372,
		1.04079482779702,1.22211204078486,1.30113287432829,0.914095057763804,1.69495025317372,1.17872570447206,
	};

	constexpr int llt_m = 6;
	constexpr int llt_row_ld = 10;
	constexpr int llt_col_ld = 9;
	const aris::dynamic::Stride llt_stride{2, 11};

	std::vector<double> llt_row_pad(static_cast<std::size_t>(llt_m * llt_row_ld), guard);
	std::vector<double> llt_col_pad(static_cast<std::size_t>(llt_col_ld * llt_m), guard);
	std::vector<double> llt_stride_pad(66, guard);
	aris::dynamic::s_mc(llt_m, llt_m, llt_a, llt_m, llt_row_pad.data(), llt_row_ld);
	aris::dynamic::s_mc(llt_m, llt_m, llt_a, llt_m, llt_col_pad.data(), aris::dynamic::ColMajor{llt_col_ld});
	aris::dynamic::s_mc(llt_m, llt_m, llt_a, llt_m, llt_stride_pad.data(), llt_stride);

	auto run_llt_type_case = [&](const std::vector<double> &a_store, auto a_t, auto l_t, const std::string &tag) {
		SCOPED_TRACE(tag);
		auto a_before = a_store;
		auto a_dense = copy_with_type(llt_m, llt_m, a_store.data(), a_t);
		std::vector<double> l_buf(128, guard);
		auto l_mask = build_mask_matrix(llt_m, llt_m, l_t, l_buf.size());
		aris::dynamic::s_llt(llt_m, a_store.data(), a_t, l_buf.data(), l_t);
		expect_full_unchanged(a_before, a_store, tag + "-A");
		expect_guard_unchanged(l_buf, l_mask, tag + "-L");
		auto l_dense = copy_with_type(llt_m, llt_m, l_buf.data(), l_t);

		for (int i = 0; i < llt_m; ++i) {
			for (int j = i + 1; j < llt_m; ++j) {
				l_dense[i * llt_m + j] = 0.0;
			}
		}

		std::vector<double> reconstructed(llt_m * llt_m, 0.0);
		aris::dynamic::s_mm(llt_m, llt_m, llt_m, l_dense.data(), llt_m, l_dense.data(), aris::dynamic::T(llt_m), reconstructed.data(), llt_m);
		expect_matrix_near(reconstructed.data(), a_dense.data(), llt_m, llt_m, 1e-8);
	};

	run_llt_type_case(llt_row_pad, llt_row_ld, llt_row_ld + 1, "llt-row-major-padded");
	run_llt_type_case(llt_col_pad, aris::dynamic::ColMajor{llt_col_ld}, aris::dynamic::ColMajor{llt_col_ld + 1}, "llt-col-major-padded");
	run_llt_type_case(llt_stride_pad, llt_stride, aris::dynamic::Stride{2, 13}, "llt-stride-padded");

	// Householder coverage with mismatched matrix type parameters.
	const double hh_a[12]{
		1.0, 2.0, 3.0,
		0.0, 1.0, 4.0,
		5.0, 6.0, 0.0,
		2.0, 1.0, 1.0,
	};
	constexpr int hh_m = 4;
	constexpr int hh_n = 3;

	constexpr int hh_row_ld = 9;
	constexpr int hh_col_ld = 7;
	const aris::dynamic::Stride hh_stride{2, 11};

	std::vector<double> hh_row_pad(static_cast<std::size_t>(hh_m * hh_row_ld), guard);
	std::vector<double> hh_col_pad(static_cast<std::size_t>(hh_col_ld * hh_n), guard);
	std::vector<double> hh_stride_pad(64, guard);
	aris::dynamic::s_mc(hh_m, hh_n, hh_a, hh_n, hh_row_pad.data(), hh_row_ld);
	aris::dynamic::s_mc(hh_m, hh_n, hh_a, hh_n, hh_col_pad.data(), aris::dynamic::ColMajor{hh_col_ld});
	aris::dynamic::s_mc(hh_m, hh_n, hh_a, hh_n, hh_stride_pad.data(), hh_stride);

	auto run_householder_type_case = [&](const std::vector<double> &a_store, auto a_t, auto u_t, auto tau_t, const std::string &tag) {
		SCOPED_TRACE(tag);
		auto a_before = a_store;
		auto a_dense = copy_with_type(hh_m, hh_n, a_store.data(), a_t);
		std::vector<double> u_buf(128, guard);
		std::vector<double> tau_buf(16, guard);
		auto u_mask = build_mask_matrix(hh_m, hh_n, u_t, u_buf.size());
		auto tau_mask = build_mask_vector(hh_n, tau_t, tau_buf.size());
		std::vector<double> q(hh_m * hh_m, 0.0);
		std::vector<double> r(hh_m * hh_n, 0.0);
		std::vector<double> reconstructed(hh_m * hh_n, 0.0);

		aris::dynamic::s_householder_ut(hh_m, hh_n, a_store.data(), a_t, u_buf.data(), u_t, tau_buf.data(), tau_t);
		expect_full_unchanged(a_before, a_store, tag + "-A");
		expect_guard_unchanged(u_buf, u_mask, tag + "-U");
		expect_guard_unchanged(tau_buf, tau_mask, tag + "-tau");
		aris::dynamic::s_householder_ut2qr(hh_m, hh_n, u_buf.data(), u_t, tau_buf.data(), tau_t, q.data(), hh_m, r.data(), hh_n);
		aris::dynamic::s_mm(hh_m, hh_n, hh_m, q.data(), hh_m, r.data(), hh_n, reconstructed.data(), hh_n);

		expect_matrix_near(reconstructed.data(), a_dense.data(), hh_m, hh_n, 1e-8);
		expect_orthogonal(q.data(), hh_m, 1e-8);

		const double x_true[3]{0.3, -1.2, 2.1};
		double b[4]{};
		double x_solved[4]{};
		aris::dynamic::s_mm(hh_m, 1, hh_n, a_dense.data(), hh_n, x_true, 1, b, 1);
		aris::dynamic::s_householder_ut_sov(hh_m, hh_n, 1, u_buf.data(), u_t, tau_buf.data(), tau_t, b, 1, x_solved, 1);
		expect_matrix_near(x_solved, x_true, 1, hh_n, 1e-8);
	};

	run_householder_type_case(hh_row_pad, hh_row_ld, hh_row_ld + 2, 2, "householder-row-major-padded");
	run_householder_type_case(hh_col_pad, aris::dynamic::ColMajor{hh_col_ld}, aris::dynamic::ColMajor{hh_col_ld + 1}, 2, "householder-col-major-padded");
	run_householder_type_case(hh_stride_pad, hh_stride, aris::dynamic::Stride{2, 13}, 2, "householder-stride-padded");
}

TEST(DynamicMatrixTest, LltLegacyBandMatrices) {
	const double b_mtx[36]{
		0.854463601335834,0.915158806735392,0.81779406393944,0,0,0,
		0.915158806735392,1.51973084603713,1.08911106493401,0,0,0,
		0.81779406393944,1.08911106493401,1.00718816511107,0,0,0,
		0,0,0,1.16505945742745,0.55867090856838,1.06119850911405,
		0,0,0,0.55867090856838,0.321956437380287,0.442097312015519,
		0,0,0,1.06119850911405,0.442097312015519,1.05023839402297,
	};
	const double c_mtx[36]{
		1.09756159899368,0.494892128610864,0,0,0,0.599982575914901,
		0.494892128610864,0.564212545623307,0,0,0,0.200848874880563,
		0,0,0.979041348469538,0.638782936554467,0.663380272499309,0,
		0,0,0.638782936554467,0.791575910209436,0.454993751889092,0,
		0,0,0.663380272499309,0.454993751889092,0.470703623796929,0,
		0.599982575914901,0.200848874880563,0,0,0,1.00673482446645,
	};

	auto check = [](const double *a, const std::string &tag) {
		SCOPED_TRACE(tag);
		double l[36]{}, l_lower[36]{}, rec[36]{};
		aris::dynamic::s_llt(6, a, l);
		for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) l_lower[i * 6 + j] = (j <= i) ? l[i * 6 + j] : 0.0;
		aris::dynamic::s_mm(6, 6, 6, l_lower, 6, l_lower, aris::dynamic::T(6), rec, 6);
		expect_matrix_near(rec, a, 6, 6, 1e-8);
	};

	check(b_mtx, "llt-legacy-B");
	check(c_mtx, "llt-legacy-C");
}

TEST(DynamicMatrixTest, Qp) {
	constexpr aris::Size nG = 2;
	constexpr aris::Size nCE = 0;
	constexpr aris::Size nCI = 0;

	const double g_mat[nG * nG]{2.0, 0.0,
		0.0, 2.0};
	const double g_vec[nG]{-2.0, -4.0};
	const double dummy[1]{0.0};
	double x[nG]{};
	std::vector<double> mem(64, 0.0);

	const double ret = aris::dynamic::s_quadprog(nG, nCE, nCI,
		g_mat, g_vec,
		dummy, dummy,
		dummy, dummy,
		x, mem.data());

	EXPECT_TRUE(std::isfinite(ret));
	EXPECT_NEAR(x[0], 1.0, 1e-8);
	EXPECT_NEAR(x[1], 2.0, 1e-8);
}

TEST(DynamicMatrixTest, InterpPlane) {
	const std::array<double, 9> x{0.0, 1.0, 2.0, 0.5, 1.5, 2.5, -1.0, -0.5, 3.0};
	const std::array<double, 9> y{0.0, 1.0, 0.5, -1.0, 2.0, -0.5, 1.5, -1.5, 0.2};
	std::array<double, 9> z{};

	for (std::size_t i = 0; i < z.size(); ++i) {
		z[i] = 2.0 * x[i] - y[i] + 3.0;
	}

	double plane[4]{};
	aris::dynamic::s_interp_plane(static_cast<aris::Size>(x.size()), x.data(), y.data(), z.data(), plane);

	const double norm = std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
	EXPECT_NEAR(norm, 1.0, 1e-10);

	for (std::size_t i = 0; i < x.size(); ++i) {
		const double residual = plane[0] * x[i] + plane[1] * y[i] + plane[2] * z[i] + plane[3];
		EXPECT_NEAR(residual, 0.0, 1e-8);
	}

	const double fit_err = aris::dynamic::s_interp_plane_error(static_cast<aris::Size>(x.size()), x.data(), y.data(), z.data(), plane);
	EXPECT_NEAR(fit_err, 0.0, 1e-8);

	const double p0[3]{1.1, 0.2, -3.5};
	const double p1[3]{2.1, 0.34, -3.2};
	const double p2[3]{1.08, 3.2, -3.4};
	const double p3[3]{1.12, 0.21, -5.5};

	const double inside[3]{1.6, 1.2, -4.5};
	const double outside[3]{5.6, 1.2, -4.5};
	EXPECT_TRUE(aris::dynamic::s_is_in_parallelepiped(p0, p1, p2, p3, inside));
	EXPECT_FALSE(aris::dynamic::s_is_in_parallelepiped(p0, p1, p2, p3, outside));

	const double axis[3]{0.81, 0.0, 0.0};
	const double c_inside[3]{1.89, 0.2, -3.5};
	const double c_outside[3]{1.91, 0.2, -3.5};
	EXPECT_TRUE(aris::dynamic::s_is_in_cylinder(p0, axis, 0.3, 0.8, c_inside));
	EXPECT_FALSE(aris::dynamic::s_is_in_cylinder(p0, axis, 0.3, 0.8, c_outside));
}

} // namespace
