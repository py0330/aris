#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <cstddef>
#include <array>
#include <list>

#include "aris/dynamic/screw.hpp"

namespace aris::dynamic
{
	using double3x3 = double[3][3];

	auto inline default_pp()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_ra()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_re()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_rq()noexcept->const double* { static const double value[4]{ 0,0,0,1 }; return value; }
	auto inline default_rm()noexcept->const double* { static const double value[9]{ 1,0,0,0,1,0,0,0,1 }; return value; }
	auto inline default_pe()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_pq()noexcept->const double* { static const double value[7]{ 0,0,0,0,0,0,1 }; return value; }
	auto inline default_pa()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_ps()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_pm()noexcept->const double* { static const double value[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; return value; }

	auto inline default_vp()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_we()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_wq()noexcept->const double* { static const double value[4]{ 0,0,0,0 }; return value; }
	auto inline default_wm()noexcept->const double* { static const double value[9]{ 0,0,0,0,0,0,0,0,0 }; return value; }
	auto inline default_ve()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_vq()noexcept->const double* { static const double value[7]{ 0,0,0,0,0,0,0 }; return value; }
	auto inline default_vm()noexcept->const double* { static const double value[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; return value; }
	auto inline default_wa()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_va()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_vs()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }

	auto inline default_ap()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_xe()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_xq()noexcept->const double* { static const double value[4]{ 0,0,0,0 }; return value; }
	auto inline default_xm()noexcept->const double* { static const double value[9]{ 0,0,0,0,0,0,0,0,0 }; return value; }
	auto inline default_ae()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_aq()noexcept->const double* { static const double value[7]{ 0,0,0,0,0,0,0 }; return value; }
	auto inline default_am()noexcept->const double* { static const double value[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; return value; }
	auto inline default_xa()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_aa()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_as()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }

	auto inline default_fs()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_im()noexcept->const double* { static const double value[36]{ 0 }; return value; }
	auto inline default_iv()noexcept->const double* { static const double value[10]{ 1,0,0,0,1,1,1,0,0,0 }; return value; }
	auto inline default_i3()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }

	auto inline default_out()noexcept->double* { static thread_local double value[36]{ 0 }; return value; }

	auto inline P()noexcept->const double3x3& { static const double p[3][3]{ { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } }; return p; }
	auto inline Q()noexcept->const double3x3& { static const double q[3][3]{ { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };	return q; }

	auto s_inv_pm(const double *pm_in, double *pm_out) noexcept->void
	{
		//转置
		pm_out[0] = pm_in[0];
		pm_out[1] = pm_in[4];
		pm_out[2] = pm_in[8];
		pm_out[4] = pm_in[1];
		pm_out[5] = pm_in[5];
		pm_out[6] = pm_in[9];
		pm_out[8] = pm_in[2];
		pm_out[9] = pm_in[6];
		pm_out[10] = pm_in[10];

		//位置
		pm_out[3] = -pm_out[0] * pm_in[3] - pm_out[1] * pm_in[7] - pm_out[2] * pm_in[11];
		pm_out[7] = -pm_out[4] * pm_in[3] - pm_out[5] * pm_in[7] - pm_out[6] * pm_in[11];
		pm_out[11] = -pm_out[8] * pm_in[3] - pm_out[9] * pm_in[7] - pm_out[10] * pm_in[11];

		//其他
		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;
	}
	auto s_pm_dot_pm(const double *pm1, const double *pm2, double *pm_out) noexcept->double *
	{
		pm_out = pm_out ? pm_out : default_out();

		pm_out[0] = pm1[0] * pm2[0] + pm1[1] * pm2[4] + pm1[2] * pm2[8];
		pm_out[1] = pm1[0] * pm2[1] + pm1[1] * pm2[5] + pm1[2] * pm2[9];
		pm_out[2] = pm1[0] * pm2[2] + pm1[1] * pm2[6] + pm1[2] * pm2[10];
		pm_out[3] = pm1[0] * pm2[3] + pm1[1] * pm2[7] + pm1[2] * pm2[11] + pm1[3];

		pm_out[4] = pm1[4] * pm2[0] + pm1[5] * pm2[4] + pm1[6] * pm2[8];
		pm_out[5] = pm1[4] * pm2[1] + pm1[5] * pm2[5] + pm1[6] * pm2[9];
		pm_out[6] = pm1[4] * pm2[2] + pm1[5] * pm2[6] + pm1[6] * pm2[10];
		pm_out[7] = pm1[4] * pm2[3] + pm1[5] * pm2[7] + pm1[6] * pm2[11] + pm1[7];

		pm_out[8] = pm1[8] * pm2[0] + pm1[9] * pm2[4] + pm1[10] * pm2[8];
		pm_out[9] = pm1[8] * pm2[1] + pm1[9] * pm2[5] + pm1[10] * pm2[9];
		pm_out[10] = pm1[8] * pm2[2] + pm1[9] * pm2[6] + pm1[10] * pm2[10];
		pm_out[11] = pm1[8] * pm2[3] + pm1[9] * pm2[7] + pm1[10] * pm2[11] + pm1[11];

		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		return pm_out;
	}
	auto s_inv_pm_dot_pm(const double *inv_pm, const double *pm, double *pm_out) noexcept->double *
	{
		pm_out = pm_out ? pm_out : default_out();

		pm_out[0] = inv_pm[0] * pm[0] + inv_pm[4] * pm[4] + inv_pm[8] * pm[8];
		pm_out[1] = inv_pm[0] * pm[1] + inv_pm[4] * pm[5] + inv_pm[8] * pm[9];
		pm_out[2] = inv_pm[0] * pm[2] + inv_pm[4] * pm[6] + inv_pm[8] * pm[10];
		pm_out[3] = inv_pm[0] * (pm[3] - inv_pm[3]) + inv_pm[4] * (pm[7] - inv_pm[7]) + inv_pm[8] * (pm[11] - inv_pm[11]);

		pm_out[4] = inv_pm[1] * pm[0] + inv_pm[5] * pm[4] + inv_pm[9] * pm[8];
		pm_out[5] = inv_pm[1] * pm[1] + inv_pm[5] * pm[5] + inv_pm[9] * pm[9];
		pm_out[6] = inv_pm[1] * pm[2] + inv_pm[5] * pm[6] + inv_pm[9] * pm[10];
		pm_out[7] = inv_pm[1] * (pm[3] - inv_pm[3]) + inv_pm[5] * (pm[7] - inv_pm[7]) + inv_pm[9] * (pm[11] - inv_pm[11]);

		pm_out[8] = inv_pm[2] * pm[0] + inv_pm[6] * pm[4] + inv_pm[10] * pm[8];
		pm_out[9] = inv_pm[2] * pm[1] + inv_pm[6] * pm[5] + inv_pm[10] * pm[9];
		pm_out[10] = inv_pm[2] * pm[2] + inv_pm[6] * pm[6] + inv_pm[10] * pm[10];
		pm_out[11] = inv_pm[2] * (pm[3] - inv_pm[3]) + inv_pm[6] * (pm[7] - inv_pm[7]) + inv_pm[10] * (pm[11] - inv_pm[11]);

		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		return pm_out;
	}
	auto s_pm_dot_inv_pm(const double *pm, const double *inv_pm, double *pm_out) noexcept->double *
	{
		pm_out = pm_out ? pm_out : default_out();

		pm_out[0] = pm[0] * inv_pm[0] + pm[1] * inv_pm[1] + pm[2] * inv_pm[2];
		pm_out[1] = pm[0] * inv_pm[4] + pm[1] * inv_pm[5] + pm[2] * inv_pm[6];
		pm_out[2] = pm[0] * inv_pm[8] + pm[1] * inv_pm[9] + pm[2] * inv_pm[10];
		pm_out[3] = -pm_out[0] * inv_pm[3] - pm_out[1] * inv_pm[7] - pm_out[2] * inv_pm[11] + pm[3];

		pm_out[4] = pm[4] * inv_pm[0] + pm[5] * inv_pm[1] + pm[6] * inv_pm[2];
		pm_out[5] = pm[4] * inv_pm[4] + pm[5] * inv_pm[5] + pm[6] * inv_pm[6];
		pm_out[6] = pm[4] * inv_pm[8] + pm[5] * inv_pm[9] + pm[6] * inv_pm[10];
		pm_out[7] = -pm_out[4] * inv_pm[3] - pm_out[5] * inv_pm[7] - pm_out[6] * inv_pm[11] + pm[7];

		pm_out[8] = pm[8] * inv_pm[0] + pm[9] * inv_pm[1] + pm[10] * inv_pm[2];
		pm_out[9] = pm[8] * inv_pm[4] + pm[9] * inv_pm[5] + pm[10] * inv_pm[6];
		pm_out[10] = pm[8] * inv_pm[8] + pm[9] * inv_pm[9] + pm[10] * inv_pm[10];
		pm_out[11] = -pm_out[8] * inv_pm[3] - pm_out[9] * inv_pm[7] - pm_out[10] * inv_pm[11] + pm[11];

		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		return pm_out;
	}
	auto s_pm_dot_v3(const double *pm, const double *v3, double *v3_out) noexcept->double *
	{
		v3_out = v3_out ? v3_out : default_out();

		v3_out[0] = pm[0] * v3[0] + pm[1] * v3[1] + pm[2] * v3[2];
		v3_out[1] = pm[4] * v3[0] + pm[5] * v3[1] + pm[6] * v3[2];
		v3_out[2] = pm[8] * v3[0] + pm[9] * v3[1] + pm[10] * v3[2];

		return v3_out;
	}
	auto s_inv_pm_dot_v3(const double *inv_pm, const double *v3, double *v3_out) noexcept->double *
	{
		v3_out = v3_out ? v3_out : default_out();

		v3_out[0] = inv_pm[0] * v3[0] + inv_pm[4] * v3[1] + inv_pm[8] * v3[2];
		v3_out[1] = inv_pm[1] * v3[0] + inv_pm[5] * v3[1] + inv_pm[9] * v3[2];
		v3_out[2] = inv_pm[2] * v3[0] + inv_pm[6] * v3[1] + inv_pm[10] * v3[2];

		return v3_out;
	}

	auto s_im_dot_as(const double *im, const double *as, double * fs) noexcept->double *
	{
		fs = fs ? fs : default_out();

		const double c[3]{ im[11], im[15], im[4] };

		s_vc(3, im[0], as, fs);
		s_c3s(c, as + 3, fs);

		s_c3(c, as, fs + 3);
		s_mma(3, 1, 3, im + 21, 6, as + 3, 1, fs + 3, 1);

		return fs;
	}
	auto s_iv_dot_as(const double *iv, const double *as, double * fs) noexcept->double *
	{
		fs = fs ? fs : default_out();

		s_vc(3, iv[0], as, fs);
		s_c3s(iv + 1, as + 3, fs);

		s_c3(iv + 1, as, fs + 3);

		fs[3] += iv[4] * as[3] + iv[7] * as[4] + iv[8] * as[5];
		fs[4] += iv[7] * as[3] + iv[5] * as[4] + iv[9] * as[5];
		fs[5] += iv[8] * as[3] + iv[9] * as[4] + iv[6] * as[5];

		return fs;
	}

	auto s_cm3(const double *a, double *cm_out) noexcept->void
	{
		cm_out[0] = 0;
		cm_out[1] = -a[2];
		cm_out[2] = a[1];
		cm_out[3] = a[2];
		cm_out[4] = 0;
		cm_out[5] = -a[0];
		cm_out[6] = -a[1];
		cm_out[7] = a[0];
		cm_out[8] = 0;
	}
	auto s_c3(const double *a, const double *b, double *c_out) noexcept->void
	{
		c_out[0] = -a[2] * b[1] + a[1] * b[2];
		c_out[1] = a[2] * b[0] - a[0] * b[2];
		c_out[2] = -a[1] * b[0] + a[0] * b[1];
	}
	auto s_c3(double alpha, const double *a, const double *b, double *c_out) noexcept->void
	{
		c_out[0] = alpha * (-a[2] * b[1] + a[1] * b[2]);
		c_out[1] = alpha * (a[2] * b[0] - a[0] * b[2]);
		c_out[2] = alpha * (-a[1] * b[0] + a[0] * b[1]);
	}
	auto s_c3i(const double *a, const double *b, double *c_out) noexcept->void
	{
		c_out[0] = a[2] * b[1] - a[1] * b[2];
		c_out[1] = -a[2] * b[0] + a[0] * b[2];
		c_out[2] = a[1] * b[0] - a[0] * b[1];
	}
	auto s_c3a(const double *a, const double *b, double *c_out) noexcept->void
	{
		c_out[0] += -a[2] * b[1] + a[1] * b[2];
		c_out[1] += a[2] * b[0] - a[0] * b[2];
		c_out[2] += -a[1] * b[0] + a[0] * b[1];
	}
	auto s_c3a(double alpha, const double *a, const double *b, double *c_out) noexcept->void
	{
		c_out[0] += alpha * (-a[2] * b[1] + a[1] * b[2]);
		c_out[1] += alpha * (a[2] * b[0] - a[0] * b[2]);
		c_out[2] += alpha * (-a[1] * b[0] + a[0] * b[1]);
	}
	auto s_c3s(const double *a, const double *b, double *c_out) noexcept->void
	{
		c_out[0] -= -a[2] * b[1] + a[1] * b[2];
		c_out[1] -= a[2] * b[0] - a[0] * b[2];
		c_out[2] -= -a[1] * b[0] + a[0] * b[1];
	}
	auto s_cmf(const double *vs_in, double *cmf_out) noexcept->void
	{
		std::fill_n(cmf_out, 36, 0);

		cmf_out[6] = vs_in[5];
		cmf_out[12] = -vs_in[4];
		cmf_out[1] = -vs_in[5];
		cmf_out[13] = vs_in[3];
		cmf_out[2] = vs_in[4];
		cmf_out[8] = -vs_in[3];

		cmf_out[27] = vs_in[5];
		cmf_out[33] = -vs_in[4];
		cmf_out[22] = -vs_in[5];
		cmf_out[34] = vs_in[3];
		cmf_out[23] = vs_in[4];
		cmf_out[29] = -vs_in[3];

		cmf_out[24] = vs_in[2];
		cmf_out[30] = -vs_in[1];
		cmf_out[19] = -vs_in[2];
		cmf_out[31] = vs_in[0];
		cmf_out[20] = vs_in[1];
		cmf_out[26] = -vs_in[0];
	}
	auto s_cf(const double *vs, const double *fs, double* vfs_out) noexcept->void
	{
		s_c3(vs + 3, fs, vfs_out);
		s_c3(vs + 3, fs + 3, vfs_out + 3);
		s_c3a(vs, fs, vfs_out + 3);
	}
	auto s_cf(double alpha, const double *vs, const double *fs, double* vfs_out) noexcept->void
	{
		s_cf(vs, fs, vfs_out);
		s_nv(6, alpha, vfs_out);
	}
	auto s_cfi(const double *vs, const double *fs, double* vfs_out) noexcept->void
	{
		s_c3i(vs + 3, fs, vfs_out);
		s_c3i(vs + 3, fs + 3, vfs_out + 3);
		s_c3s(vs, fs, vfs_out + 3);
	}

	auto s_cfa(const double *vs, const double *fs, double* vfs_out) noexcept->void
	{
		s_c3a(vs + 3, fs, vfs_out);
		s_c3a(vs + 3, fs + 3, vfs_out + 3);
		s_c3a(vs, fs, vfs_out + 3);
	}
	auto s_cfa(double alpha, const double *vs, const double *fs, double* vfs_out) noexcept->void
	{
		s_c3a(alpha, vs + 3, fs, vfs_out);
		s_c3a(alpha, vs + 3, fs + 3, vfs_out + 3);
		s_c3a(alpha, vs, fs, vfs_out + 3);
	}
	auto s_cfs(const double *vs, const double *fs, double* vfs_out) noexcept->void
	{
		s_c3s(vs + 3, fs, vfs_out);
		s_c3s(vs + 3, fs + 3, vfs_out + 3);
		s_c3s(vs, fs, vfs_out + 3);
	}
	auto s_cmv(const double *vs_in, double *cmv_out) noexcept->void
	{
		std::fill_n(cmv_out, 36, 0);

		cmv_out[6] = vs_in[5];
		cmv_out[12] = -vs_in[4];
		cmv_out[1] = -vs_in[5];
		cmv_out[13] = vs_in[3];
		cmv_out[2] = vs_in[4];
		cmv_out[8] = -vs_in[3];

		cmv_out[27] = vs_in[5];
		cmv_out[33] = -vs_in[4];
		cmv_out[22] = -vs_in[5];
		cmv_out[34] = vs_in[3];
		cmv_out[23] = vs_in[4];
		cmv_out[29] = -vs_in[3];

		cmv_out[9] = vs_in[2];
		cmv_out[15] = -vs_in[1];
		cmv_out[4] = -vs_in[2];
		cmv_out[16] = vs_in[0];
		cmv_out[5] = vs_in[1];
		cmv_out[11] = -vs_in[0];
	}
	auto s_cv(const double *vs, const double *vs2, double* vvs_out) noexcept->void
	{
		s_c3(vs + 3, vs2, vvs_out);
		s_c3(vs + 3, vs2 + 3, vvs_out + 3);
		s_c3a(vs, vs2 + 3, vvs_out);
	}
	auto s_cv(double alpha, const double *vs, const double *vs2, double* vvs_out) noexcept->void
	{
		s_cv(vs, vs2, vvs_out);
		s_nv(6, alpha, vvs_out);
	}
	auto s_cvi(const double *vs, const double *vs2, double* vvs_out) noexcept->void
	{
		s_c3i(vs + 3, vs2, vvs_out);
		s_c3i(vs + 3, vs2 + 3, vvs_out + 3);
		s_c3s(vs, vs2 + 3, vvs_out);
	}
	auto s_cva(const double *vs, const double *vs2, double* vvs_out) noexcept->void
	{
		s_c3a(vs + 3, vs2, vvs_out);
		s_c3a(vs + 3, vs2 + 3, vvs_out + 3);
		s_c3a(vs, vs2 + 3, vvs_out);
	}
	auto s_cva(double alpha, const double *vs, const double *vs2, double* vvs_out) noexcept->void
	{
		s_c3a(alpha, vs + 3, vs2, vvs_out);
		s_c3a(alpha, vs + 3, vs2 + 3, vvs_out + 3);
		s_c3a(alpha, vs, vs2 + 3, vvs_out);
	}
	auto s_cvs(const double *vs, const double *vs2, double* vvs_out) noexcept->void
	{
		s_c3s(vs + 3, vs2, vvs_out);
		s_c3s(vs + 3, vs2 + 3, vvs_out + 3);
		s_c3s(vs, vs2 + 3, vvs_out);
	}

	auto s_tmf(const double *pm_in, double *tmf_out) noexcept->void
	{
		std::fill_n(tmf_out + 3, 3, 0);
		std::fill_n(tmf_out + 9, 3, 0);
		std::fill_n(tmf_out + 15, 3, 0);

		std::copy_n(&pm_in[0], 3, &tmf_out[0]);
		std::copy_n(&pm_in[4], 3, &tmf_out[6]);
		std::copy_n(&pm_in[8], 3, &tmf_out[12]);
		std::copy_n(&pm_in[0], 3, &tmf_out[21]);
		std::copy_n(&pm_in[4], 3, &tmf_out[27]);
		std::copy_n(&pm_in[8], 3, &tmf_out[33]);

		tmf_out[18] = -pm_in[11] * pm_in[4] + pm_in[7] * pm_in[8];
		tmf_out[24] = pm_in[11] * pm_in[0] - pm_in[3] * pm_in[8];
		tmf_out[30] = -pm_in[7] * pm_in[0] + pm_in[3] * pm_in[4];
		tmf_out[19] = -pm_in[11] * pm_in[5] + pm_in[7] * pm_in[9];
		tmf_out[25] = pm_in[11] * pm_in[1] - pm_in[3] * pm_in[9];
		tmf_out[31] = -pm_in[7] * pm_in[1] + pm_in[3] * pm_in[5];
		tmf_out[20] = -pm_in[11] * pm_in[6] + pm_in[7] * pm_in[10];
		tmf_out[26] = pm_in[11] * pm_in[2] - pm_in[3] * pm_in[10];
		tmf_out[32] = -pm_in[7] * pm_in[2] + pm_in[3] * pm_in[6];
	}
	auto s_tf(const double *pm, const double *fs, double *fs_out) noexcept->void
	{
		s_pm_dot_v3(pm, fs, fs_out);
		s_pm_dot_v3(pm, fs + 3, fs_out + 3);
		s_c3a(pm + 3, 4, fs_out, 1, fs_out + 3, 1);
	}
	auto s_tf(double alpha, const double *pm, const double *fs, double *fs_out) noexcept->void
	{
		s_tf(pm, fs, fs_out);
		s_nv(6, alpha, fs_out);
	}
	auto s_tfa(const double *pm, const double *fs, double *fs_out) noexcept->void
	{
		double tem[6];
		s_tf(pm, fs, tem);
		s_va(6, tem, fs_out);
	}
	auto s_tfa(double alpha, const double *pm, const double *fs, double *fs_out) noexcept->void
	{
		double tem[6];
		s_tf(pm, fs, tem);
		s_va(6, alpha, tem, fs_out);
	}
	auto s_inv_tf(const double *inv_pm, const double *fs, double *fs_out) noexcept->void
	{
		s_c3i(inv_pm + 3, 4, fs, 1, fs_out, 1);
		s_va(3, fs + 3, fs_out);

		s_inv_pm_dot_v3(inv_pm, fs_out, fs_out + 3);
		s_inv_pm_dot_v3(inv_pm, fs, fs_out);
	}
	auto s_inv_tf(double alpha, const double *inv_pm, const double *fs, double *fs_out) noexcept->void
	{
		s_inv_tf(inv_pm, fs, fs_out);
		s_nv(6, alpha, fs_out);
	}
	auto s_inv_tfa(const double *inv_pm, const double *fs, double *fs_out) noexcept->void
	{
		double tem[6];
		s_inv_tf(inv_pm, fs, tem);
		s_va(6, tem, fs_out);
	}
	auto s_inv_tfa(double alpha, const double *inv_pm, const double *fs, double *fs_out) noexcept->void
	{
		double tem[6];
		s_inv_tf(inv_pm, fs, tem);
		s_va(6, alpha, tem, fs_out);
	}
	auto s_tmv(const double *pm, double *tmv_out) noexcept->void
	{
		std::fill_n(tmv_out + 18, 3, 0);
		std::fill_n(tmv_out + 24, 3, 0);
		std::fill_n(tmv_out + 30, 3, 0);

		std::copy_n(&pm[0], 3, &tmv_out[0]);
		std::copy_n(&pm[4], 3, &tmv_out[6]);
		std::copy_n(&pm[8], 3, &tmv_out[12]);
		std::copy_n(&pm[0], 3, &tmv_out[21]);
		std::copy_n(&pm[4], 3, &tmv_out[27]);
		std::copy_n(&pm[8], 3, &tmv_out[33]);

		tmv_out[3] = -pm[11] * pm[4] + pm[7] * pm[8];
		tmv_out[9] = pm[11] * pm[0] - pm[3] * pm[8];
		tmv_out[15] = -pm[7] * pm[0] + pm[3] * pm[4];
		tmv_out[4] = -pm[11] * pm[5] + pm[7] * pm[9];
		tmv_out[10] = pm[11] * pm[1] - pm[3] * pm[9];
		tmv_out[16] = -pm[7] * pm[1] + pm[3] * pm[5];
		tmv_out[5] = -pm[11] * pm[6] + pm[7] * pm[10];
		tmv_out[11] = pm[11] * pm[2] - pm[3] * pm[10];
		tmv_out[17] = -pm[7] * pm[2] + pm[3] * pm[6];
	}
	auto s_tv(const double *pm, const double *vs, double *vs_out) noexcept->void
	{
		s_pm_dot_v3(pm, vs, vs_out);
		s_pm_dot_v3(pm, vs + 3, vs_out + 3);
		s_c3a(pm + 3, 4, vs_out + 3, 1, vs_out, 1);
	}
	auto s_tv(double alpha, const double *pm, const double *vs, double *vs_out) noexcept->void
	{
		s_tv(pm, vs, vs_out);
		s_nv(6, alpha, vs_out);
	}
	auto s_tva(const double *pm, const double *vs, double *vs_out) noexcept->void
	{
		double tem[6];
		s_tv(pm, vs, tem);
		s_va(6, tem, vs_out);
	}
	auto s_tva(double alpha, const double *pm, const double *vs, double *vs_out) noexcept->void
	{
		double tem[6];
		s_tv(pm, vs, tem);
		s_va(6, alpha, tem, vs_out);
	}
	auto s_inv_tv(const double *inv_pm, const double *vs, double *vs_out) noexcept->void
	{
		s_c3i(inv_pm + 3, 4, vs + 3, 1, vs_out + 3, 1);
		s_va(3, vs, vs_out + 3);

		s_inv_pm_dot_v3(inv_pm, vs_out + 3, vs_out);
		s_inv_pm_dot_v3(inv_pm, vs + 3, vs_out + 3);
	}
	auto s_inv_tv(double alpha, const double *inv_pm, const double *vs, double *vs_out) noexcept->void
	{
		s_inv_tv(inv_pm, vs, vs_out);
		s_nv(6, alpha, vs_out);
	}
	auto s_inv_tva(const double *inv_pm, const double *vs, double *vs_out) noexcept->void
	{
		double tem[6];
		s_inv_tv(inv_pm, vs, tem);
		s_va(6, tem, vs_out);
	}
	auto s_inv_tva(const double *inv_pm, const double *vs, Size vs_ld, double *vs_out, Size vs_out_ld) noexcept->void
	{
		double tem[6];
		s_inv_tv(inv_pm, vs, vs_ld, tem, 1);
		s_va(6, tem, 1, vs_out, vs_out_ld);
	}
	auto s_inv_tva(double alpha, const double *inv_pm, const double *vs, double *vs_out) noexcept->void
	{
		double tem[6];
		s_inv_tv(inv_pm, vs, tem);
		s_va(6, alpha, tem, vs_out);
	}
	auto s_inv_tva(double alpha, const double *inv_pm, const double *vs, Size vs_ld, double *vs_out, Size vs_out_ld) noexcept->void
	{
		double tem[6];
		s_inv_tv(inv_pm, vs, vs_ld, tem, 1);
		s_va(6, alpha, tem, 1, vs_out, vs_out_ld);
	}

	auto s_ra2rm(const double *ra_in, double *rm_out, Size rm_ld)noexcept->double *
	{
		// 补充默认参数 //
		ra_in = ra_in ? ra_in : default_ra();
		rm_out = rm_out ? rm_out : default_out();

		double theta = std::sqrt(ra_in[0] * ra_in[0] + ra_in[1] * ra_in[1] + ra_in[2] * ra_in[2]);

		const double &a = ra_in[0];
		const double &b = ra_in[1];
		const double &c = ra_in[2];

		const double A = s_sinx_over_x(theta);
		const double B = s_one_minus_cosx_over_square_x(theta);

		rm_out[0] = 1 - (b * b + c * c)*B;
		rm_out[1] = -c * A + a * b * B;
		rm_out[2] = b * A + a * c * B;

		rm_out[rm_ld] = c * A + a * b * B;
		rm_out[rm_ld + 1] = 1 - (a * a + c * c) * B;
		rm_out[rm_ld + 2] = -a * A + b * c * B;

		rm_out[rm_ld + rm_ld] = -b * A + a * c * B;
		rm_out[rm_ld + rm_ld + 1] = a * A + b * c * B;
		rm_out[rm_ld + rm_ld + 2] = 1 - (a * a + b * b) * B;

		return rm_out;
	}
	auto s_rm2ra(const double *rm_in, double *ra_out, Size rm_ld)noexcept->double *
	{
		// 补充默认参数 //
		rm_in = rm_in ? rm_in : default_rm();
		ra_out = ra_out ? ra_out : default_out();

		double rq[4];
		s_rm2rq(rm_in, rq, rm_ld);

		// theta here is (-pi, pi]
		// thus sin(theta/2) only has one zero point
		double theta = atan2(s_norm(3, rq), rq[3]) * 2;
		double coe = 2.0 / s_sinx_over_x(theta / 2.0);
		s_nv(3, coe, rq);

		s_vc(3, rq, ra_out);

		return ra_out;
	}
	auto s_re2rm(const double *re_in, double *rm_out, const char *eu_type_in, Size rm_ld) noexcept->double *
	{
		// 补充默认参数 //
		re_in = re_in ? re_in : default_re();
		rm_out = rm_out ? rm_out : default_out();

		// 正式开始计算 //
		double Abb, Add, Abd, Adb;
		double Bac, Bae, Bdc, Bde;
		double Cbb, Cee, Cbe, Ceb;
		double s_, c_;

		const Size a = eu_type_in[0] - '1';
		const Size b = eu_type_in[1] - '1';
		const Size c = eu_type_in[2] - '1';
		const Size d = 3 - a - b;
		const Size e = 3 - b - c;

		c_ = std::cos(re_in[0]);
		s_ = std::sin(re_in[0]);
		Abb = c_;
		Add = Abb;
		Abd = P()[b][d] * s_;
		Adb = -Abd;

		s_ = std::sin(re_in[1]);
		c_ = std::cos(re_in[1]);
		Bac = P()[a][c] * s_ + Q()[a][c] * c_;
		Bae = P()[a][e] * s_ + Q()[a][e] * c_;
		Bdc = P()[d][c] * s_ + Q()[d][c] * c_;
		Bde = P()[d][e] * s_ + Q()[d][e] * c_;

		c_ = std::cos(re_in[2]);
		s_ = std::sin(re_in[2]);
		Cbb = c_;
		Cee = Cbb;
		Cbe = P()[b][e] * s_;
		Ceb = -Cbe;

		rm_out[a * rm_ld + c] = Bac;
		rm_out[a * rm_ld + b] = Bae * Ceb;
		rm_out[a * rm_ld + e] = Bae * Cee;
		rm_out[b * rm_ld + c] = Abd * Bdc;
		rm_out[b * rm_ld + b] = Abb * Cbb + Abd * Bde * Ceb;
		rm_out[b * rm_ld + e] = Abb * Cbe + Abd * Bde * Cee;
		rm_out[d * rm_ld + c] = Add * Bdc;
		rm_out[d * rm_ld + b] = Adb * Cbb + Add * Bde * Ceb;
		rm_out[d * rm_ld + e] = Adb * Cbe + Add * Bde * Cee;

		return rm_out;
	}
	auto s_rm2re(const double *rm_in, double *re_out, const char *eu_type_in, Size rm_ld) noexcept->double *
	{
		// 补充默认参数 //
		rm_in = rm_in ? rm_in : default_rm();
		re_out = re_out ? re_out : default_out();

		// 对于aba形式的欧拉角，其取值范围为：
		// [0,2PI),[0,PI],[0,2PI)
		// 其另外一个解为：a+PI, 2*PI-b, c+PI
		// 对于abc形式的欧拉角，其取值范围为：
		// [0,2PI),[-PI/2,PI/2],[0,2PI)
		// 其另外一个解为：a+PI, PI-b, c+PI

		// 正式开始计算 //
		const Size a = eu_type_in[0] - '1';
		const Size b = eu_type_in[1] - '1';
		const Size c = eu_type_in[2] - '1';
		const Size d = 3 - a - b;
		const Size e = 3 - b - c;

		// 计算phi2 //
		double s_ = std::sqrt((rm_in[rm_ld * a + b] * rm_in[rm_ld * a + b] + rm_in[rm_ld * a + e] * rm_in[rm_ld * a + e]
			+ rm_in[rm_ld * b + c] * rm_in[rm_ld * b + c] + rm_in[rm_ld * d + c] * rm_in[rm_ld * d + c]) / 2);

		double c_ = rm_in[rm_ld * a + c];
		re_out[1] = (a == c ? std::atan2(s_, c_) : std::atan2(P()[a][c] * c_, s_));

		// 计算phi1和phi3 //
		double phi13 = std::atan2(rm_in[rm_ld * b + e] - rm_in[rm_ld * d + b], rm_in[rm_ld * b + b] + rm_in[rm_ld * d + e]);
		double phi31 = std::atan2(rm_in[rm_ld * b + e] + rm_in[rm_ld * d + b], rm_in[rm_ld * b + b] - rm_in[rm_ld * d + e]);

		re_out[0] = P()[b][d] * (phi13 - phi31) / 2;
		re_out[2] = P()[b][e] * (phi13 + phi31) / 2;

		// 检查 //
		double sig[4];
		sig[0] = (P()[a][e] + Q()[a][e])*P()[e][b] * rm_in[a * rm_ld + b] * std::sin(re_out[2]);
		sig[1] = (P()[a][e] + Q()[a][e])*rm_in[rm_ld * a + e] * std::cos(re_out[2]);
		sig[2] = (P()[d][c] + Q()[d][c])*P()[b][d] * rm_in[b * rm_ld + c] * std::sin(re_out[0]);
		sig[3] = (P()[d][c] + Q()[d][c])*rm_in[rm_ld * d + c] * std::cos(re_out[0]);

		if (*std::max_element(sig, sig + rm_ld, [](double d1, double d2) {return (std::abs(d1) < std::abs(d2)); })<0)
		{
			re_out[0] += PI;
			re_out[2] += PI;
		}

		re_out[0] = (re_out[0] < 0 ? re_out[0] + 2 * PI : re_out[0]);
		re_out[2] = (re_out[2] < 0 ? re_out[2] + 2 * PI : re_out[2]);

		return re_out;
	}
	auto s_rq2rm(const double *rq_in, double *rm_out, Size rm_ld) noexcept->double *
	{
		// 补充默认参数 //
		rq_in = rq_in ? rq_in : default_rq();
		rm_out = rm_out ? rm_out : default_out();

		// 正式开始计算 //
		rm_out[0 * rm_ld + 0] = 1 - 2 * rq_in[1] * rq_in[1] - 2 * rq_in[2] * rq_in[2];
		rm_out[0 * rm_ld + 1] = 2 * rq_in[0] * rq_in[1] - 2 * rq_in[3] * rq_in[2];
		rm_out[0 * rm_ld + 2] = 2 * rq_in[0] * rq_in[2] + 2 * rq_in[3] * rq_in[1];

		rm_out[1 * rm_ld + 0] = 2 * rq_in[0] * rq_in[1] + 2 * rq_in[3] * rq_in[2];
		rm_out[1 * rm_ld + 1] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[2] * rq_in[2];
		rm_out[1 * rm_ld + 2] = 2 * rq_in[1] * rq_in[2] - 2 * rq_in[3] * rq_in[0];

		rm_out[2 * rm_ld + 0] = 2 * rq_in[0] * rq_in[2] - 2 * rq_in[3] * rq_in[1];
		rm_out[2 * rm_ld + 1] = 2 * rq_in[1] * rq_in[2] + 2 * rq_in[3] * rq_in[0];
		rm_out[2 * rm_ld + 2] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[1] * rq_in[1];

		return rm_out;
	}
	auto s_rm2rq(const double *rm_in, double *rq_out, Size rm_ld) noexcept->double *
	{
		// 补充默认参数 //
		rm_in = rm_in ? rm_in : default_rm();
		rq_out = rq_out ? rq_out : default_out();

		// 正式开始计算 //
		static const double T[4][4]{ { 0,1,1,-1 },{ 1,0,1,-1 },{ 1,1,0,-1 },{ -1,-1,-1,0 } };
		static const int P[4][4]{ { -1,0,0,2 },{ 1,-1,1,0 },{ 2,2,-1,1 },{ 2,0,1,-1 } };
		static const int Q[4][4]{ { -1,1,2,1 },{ 0,-1,2,2 },{ 0,1,-1,0 },{ 1,2,0,-1 } };

		double qt_square[4];

		qt_square[0] = (1 + rm_in[0] - rm_in[rm_ld + 1] - rm_in[2 * rm_ld + 2]) / 4;
		qt_square[1] = (1 + rm_in[rm_ld + 1] - rm_in[0] - rm_in[2 * rm_ld + 2]) / 4;
		qt_square[2] = (1 + rm_in[2 * rm_ld + 2] - rm_in[0] - rm_in[rm_ld + 1]) / 4;
		qt_square[3] = (1 + rm_in[0] + rm_in[rm_ld + 1] + rm_in[2 * rm_ld + 2]) / 4;

		int i = static_cast<int>(std::max_element(qt_square, qt_square + 4) - qt_square);
		rq_out[i] = std::sqrt(qt_square[i]);

		int jkl[3]{ (i + 1) % 4 ,(i + 2) % 4 ,(i + 3) % 4 };
		for (auto m : jkl)rq_out[m] = (rm_in[P[i][m] * rm_ld + Q[i][m]] + T[i][m] * rm_in[Q[i][m] * rm_ld + P[i][m]]) / 4.0 / rq_out[i];

		// 将rq[3]置为正
		for (auto m = 0; m < 4; ++m)rq_out[m] = rq_out[3] < 0 ? -rq_out[m] : rq_out[m];

		return rq_out;
	}
	auto s_pp2pm(const double *pp_in, double *pm_out) noexcept->double *
	{
		// 补充默认参数 //
		pp_in = pp_in ? pp_in : default_pp();
		pm_out = pm_out ? pm_out : default_out();

		// 正式开始计算 //
		pm_out[3] = pp_in[0];
		pm_out[7] = pp_in[1];
		pm_out[11] = pp_in[2];

		return pm_out;
	}
	auto s_pm2pp(const double *pm_in, double *pp_out) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		pp_out = pp_out ? pp_out : default_out();

		// 正式开始计算 //
		pp_out[0] = pm_in[3];
		pp_out[1] = pm_in[7];
		pp_out[2] = pm_in[11];

		return pp_out;
	}
	auto s_re2pm(const double *re_in, double *pm_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		re_in = re_in ? re_in : default_re();
		pm_out = pm_out ? pm_out : default_out();

		// 正式开始计算 //
		s_re2rm(re_in, pm_out, eu_type_in, 4);

		return pm_out;
	};
	auto s_pm2re(const double *pm_in, double *re_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		re_out = re_out ? re_out : default_out();

		// 正式开始计算 //
		s_rm2re(pm_in, re_out, eu_type_in, 4);

		return re_out;
	};
	auto s_ra2pm(const double *ra_in, double *pm_out) noexcept->double * { return s_ra2rm(ra_in, pm_out, 4); };
	auto s_pm2ra(const double *pm_in, double *ra_out) noexcept->double * { return s_rm2ra(pm_in, ra_out, 4); };
	auto s_rq2pm(const double *rq_in, double *pm_out) noexcept->double * { return s_rq2rm(rq_in, pm_out, 4); };
	auto s_pm2rq(const double *pm_in, double *rq_out) noexcept->double * { return s_rm2rq(pm_in, rq_out, 4); }
	auto s_rm2pm(const double *rm_in, double *pm_out, Size rm_ld) noexcept->double *
	{
		// 补充默认参数 //
		rm_in = rm_in ? rm_in : default_rm();
		pm_out = pm_out ? pm_out : default_out();

		// 正式开始计算 //
		std::copy(rm_in, rm_in + 3, pm_out);
		std::copy(rm_in + rm_ld, rm_in + rm_ld + 3, pm_out + 4);
		std::copy(rm_in + rm_ld * 2, rm_in + rm_ld * 2 + 3, pm_out + 8);

		return pm_out;
	}
	auto s_pm2rm(const double *pm_in, double *rm_out, Size rm_ld) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		rm_out = rm_out ? rm_out : default_out();

		// 正式开始计算 //
		std::copy(pm_in, pm_in + 3, rm_out);
		std::copy(pm_in + 4, pm_in + 7, rm_out + rm_ld);
		std::copy(pm_in + 8, pm_in + 11, rm_out + rm_ld * 2);

		return rm_out;
	}
	auto s_pe2pm(const double *pe_in, double *pm_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		pe_in = pe_in ? pe_in : default_pe();
		pm_out = pm_out ? pm_out : default_out();

		// 正式开始计算 //
		s_pp2pm(pe_in, pm_out);
		s_re2pm(pe_in + 3, pm_out, eu_type_in);

		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		return pm_out;
	}
	auto s_pm2pe(const double *pm_in, double *pe_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		pe_out = pe_out ? pe_out : default_out();

		// 正式开始计算 //
		s_pm2pp(pm_in, pe_out);
		s_pm2re(pm_in, pe_out + 3, eu_type_in);

		return pe_out;
	}
	auto s_pq2pm(const double *pq_in, double *pm_out) noexcept->double *
	{
		// 补充默认参数 //
		pq_in = pq_in ? pq_in : default_pq();
		pm_out = pm_out ? pm_out : default_out();

		// 正式开始计算 //
		s_pp2pm(pq_in, pm_out);
		s_rq2pm(pq_in + 3, pm_out);

		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		return pm_out;
	}
	auto s_pm2pq(const double *pm_in, double *pq_out) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		pq_out = pq_out ? pq_out : default_out();

		// 正式开始计算 //
		s_pm2pp(pm_in, pq_out);
		s_pm2rq(pm_in, pq_out + 3);

		return pq_out;
	}
	auto s_pa2pm(const double *pa_in, double *pm_out) noexcept->double *
	{
		// 补充默认参数 //
		pa_in = pa_in ? pa_in : default_pa();
		pm_out = pm_out ? pm_out : default_out();

		// 正式开始计算 //
		s_pp2pm(pa_in, pm_out);
		s_ra2pm(pa_in + 3, pm_out);

		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		return pm_out;
	}
	auto s_pm2pa(const double *pm_in, double *pa_out) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		pa_out = pa_out ? pa_out : default_out();

		// 正式开始计算 //
		s_pm2pp(pm_in, pa_out);
		s_pm2ra(pm_in, pa_out + 3);

		return pa_out;
	}
	auto s_ps2pm(const double *ps_in, double *pm_out) noexcept->double *
	{
		// 补充默认参数 //
		ps_in = ps_in ? ps_in : default_ps();
		pm_out = pm_out ? pm_out : default_out();

		// 正式开始计算 //
		s_ra2pm(ps_in + 3, pm_out);
		// 计算螺旋前进的位移 //
		// vh 是螺线线方向的速度
		// vt 是切线方向的速度

		// norm(w)^2
		const double n_square = ps_in[3] * ps_in[3] + ps_in[4] * ps_in[4] + ps_in[5] * ps_in[5];
		// norm(w)
		const double n = std::sqrt(n_square);

		// vh is parallel part of v with w
		double ratio = n < 1e-8 ? 0.0 : (ps_in[0] * ps_in[3] + ps_in[1] * ps_in[4] + ps_in[2] * ps_in[5]) / n_square;
		const double vh[3]{ ratio*ps_in[3],ratio*ps_in[4],ratio*ps_in[5] };

		// vt is vertical part of v with w
		const double vt[3]{ ps_in[0] - vh[0],ps_in[1] - vh[1],ps_in[2] - vh[2] };

		// p = vh + sin(n) / n * vt + (1-cos(n)) / n^2 * w x v
		double w_cross_v[3];
		s_c3(ps_in + 3, ps_in, w_cross_v);

		auto r1 = s_sinx_over_x(n);
		auto r2 = s_one_minus_cosx_over_square_x(n);

		pm_out[3] = vh[0] + vt[0] * r1 + w_cross_v[0] * r2;
		pm_out[7] = vh[1] + vt[1] * r1 + w_cross_v[1] * r2;
		pm_out[11] = vh[2] + vt[2] * r1 + w_cross_v[2] * r2;

		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		return pm_out;
	}
	auto s_pm2ps(const double *pm_in, double *ps_out) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		ps_out = ps_out ? ps_out : default_out();

		// 正式开始计算 //
		s_pm2ra(pm_in, ps_out + 3);

		// v = vh + vt
		// vh= (w * v) / |w|^2 * w
		//                  [ w(0)*w(0)  w(0)*w(1)  w(0)*w(2) ]
		//   = 1 / |w|^2 *  | w(1)*w(0)  w(1)*w(1)  w(1)*w(2) |  *  v
		//                  [ w(2)*w(0)  w(2)*w(1)  w(2)*w(2) ]
		//   = 1 / |w|^2 * A * v
		// 
		// vt= v - vh
		//
		// p = vh + sin(|w|) / |w| * vt + (1 - cos(|w|)) / |w|^2 * w x v 
		//   = sin(|w|)/|w| * v + [1 - sin(|w|)/|w|] / |w|^2 * A * v + (1 - cos(|w|)) / |w|^2 * w x v 
		//   = [ sin(|w|)/|w|*I + [|w| - sin(|w|)] / |w|^3 * A + (1 - cos(|w|)) / |w|^2 * w x ] * v
		//   = [a*A + b*wx + c*I] * v
		//
		// 其中：
		// a = [|w| - sin(|w|)] / |w|^3
		// b = (1 - cos(|w|)) / |w|^2
		// c = sin(|w|)/|w|
		// 
		// 在趋向于0时：
		// a = 1 / 6
		// b = 0.5
		// c = 1.0
		// 
		// 其中a趋向于0的判断应该在1e-3左右
		//

		// norm(w)^2
		const double n_square = ps_out[3] * ps_out[3] + ps_out[4] * ps_out[4] + ps_out[5] * ps_out[5];
		// norm(w)
		const double n = std::sqrt(n_square);

		const double a = n < 1e-3 ? 1.0 / 6.0 : (1.0 - std::sin(n) / n) / n / n;
		const double b = s_one_minus_cosx_over_square_x(n);
		const double c = s_sinx_over_x(n);

		const double T[9]
		{
			a * ps_out[3] * ps_out[3] + c, a * ps_out[3] * ps_out[4] - b * ps_out[5], a * ps_out[3] * ps_out[5] + b * ps_out[4],
			a * ps_out[4] * ps_out[3] + b * ps_out[5], a * ps_out[4] * ps_out[4] + c, a * ps_out[4] * ps_out[5] - b * ps_out[3],
			a * ps_out[5] * ps_out[3] - b * ps_out[4], a * ps_out[5] * ps_out[4] + b * ps_out[3], a * ps_out[5] * ps_out[5] + c,
		};

		const double det = T[0] * T[4] * T[8] - T[0] * T[5] * T[7] + T[1] * T[5] * T[6] - T[1] * T[3] * T[8] + T[2] * T[3] * T[7] - T[2] * T[4] * T[6];
		ps_out[0] = (pm_in[3] * T[4] * T[8] - pm_in[3] * T[5] * T[7] + T[1] * T[5] * pm_in[11] - T[1] * pm_in[7] * T[8] + T[2] * pm_in[7] * T[7] - T[2] * T[4] * pm_in[11]) / det;
		ps_out[1] = (T[0] * pm_in[7] * T[8] - T[0] * T[5] * pm_in[11] + pm_in[3] * T[5] * T[6] - pm_in[3] * T[3] * T[8] + T[2] * T[3] * pm_in[11] - T[2] * pm_in[7] * T[6]) / det;
		ps_out[2] = (T[0] * T[4] * pm_in[11] - T[0] * pm_in[7] * T[7] + T[1] * pm_in[7] * T[6] - T[1] * T[3] * pm_in[11] + pm_in[3] * T[3] * T[7] - pm_in[3] * T[4] * T[6]) / det;

		return ps_out;
	}

	auto s_we2wa(const double *re_in, const double *we_in, double *wa_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		re_in = re_in ? re_in : default_re();
		we_in = we_in ? we_in : default_we();
		wa_out = wa_out ? wa_out : default_out();

		// 正式开始计算 //
		const Size a = eu_type_in[0] - '1';
		const Size b = eu_type_in[1] - '1';
		const Size c = eu_type_in[2] - '1';
		const Size d = 3 - a - b;
		const Size e = 3 - b - c;

		double axis[3][3];

		axis[a][0] = 1.0;
		axis[b][0] = 0.0;
		axis[d][0] = 0.0;

		axis[a][1] = 0;
		axis[b][1] = std::cos(re_in[0]);
		axis[d][1] = P()[d][b] * std::sin(re_in[0]);

		axis[a][2] = c == a ? std::cos(re_in[1]) : P()[a][d] * std::sin(re_in[1]);
		axis[b][2] = c == a ? -std::sin(re_in[0])*std::sin(re_in[1]) : P()[b][d] * std::sin(re_in[0])* std::cos(re_in[1]);
		axis[d][2] = c == a ? P()[d][a] * std::cos(re_in[0])* std::sin(re_in[1]) : std::cos(re_in[0])* std::cos(re_in[1]);

		s_mm(3, 1, 3, *axis, 3, we_in, 1, wa_out, 1);

		return wa_out;
	}
	auto s_wa2we(const double *wa_in, const double *re_in, double *we_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		wa_in = wa_in ? wa_in : default_wa();
		re_in = re_in ? re_in : default_re();
		we_out = we_out ? we_out : default_out();

		// 正式开始计算 //
		const Size a = eu_type_in[0] - '1';
		const Size b = eu_type_in[1] - '1';
		const Size c = eu_type_in[2] - '1';
		const Size d = 3 - a - b;
		const Size e = 3 - b - c;

		double axis[3][3];

		axis[a][0] = 1.0;
		axis[b][0] = 0.0;
		axis[d][0] = 0.0;

		axis[a][1] = 0;
		axis[b][1] = std::cos(re_in[0]);
		axis[d][1] = P()[d][b] * std::sin(re_in[0]);

		axis[a][2] = c == a ? std::cos(re_in[1]) : P()[a][d] * std::sin(re_in[1]);
		axis[b][2] = c == a ? -std::sin(re_in[0])*std::sin(re_in[1]) : P()[b][d] * std::sin(re_in[0])* std::cos(re_in[1]);
		axis[d][2] = c == a ? P()[d][a] * std::cos(re_in[0])* std::sin(re_in[1]) : std::cos(re_in[0])* std::cos(re_in[1]);

		we_out[1] = (wa_in[b] * axis[d][2] - wa_in[d] * axis[b][2]) / (axis[b][1] * axis[d][2] - axis[d][1] * axis[b][2]);
		we_out[2] = (wa_in[d] * axis[b][1] - wa_in[b] * axis[d][1]) / (axis[d][2] * axis[b][1] - axis[b][2] * axis[d][1]);
		we_out[0] = (wa_in[a] - axis[a][2] * we_out[2]);

		return we_out;
	}
	auto s_wq2wa(const double *rq_in, const double *wq_in, double *wa_out) noexcept->double *
	{
		// 补充默认参数 //
		rq_in = rq_in ? rq_in : default_rq();
		wq_in = wq_in ? wq_in : default_wq();
		wa_out = wa_out ? wa_out : default_out();

		// 正式开始计算 //
		double p11 = 2 * wq_in[0] * rq_in[0];
		double p22 = 2 * wq_in[1] * rq_in[1];
		double p33 = 2 * wq_in[2] * rq_in[2];
		double p12 = wq_in[0] * rq_in[1] + rq_in[0] * wq_in[1];
		double p13 = wq_in[0] * rq_in[2] + rq_in[0] * wq_in[2];
		double p23 = wq_in[1] * rq_in[2] + rq_in[1] * wq_in[2];
		double p41 = wq_in[3] * rq_in[0] + rq_in[3] * wq_in[0];
		double p42 = wq_in[3] * rq_in[1] + rq_in[3] * wq_in[1];
		double p43 = wq_in[3] * rq_in[2] + rq_in[3] * wq_in[2];

		double rm[3][3];
		s_rq2rm(rq_in, *rm);

		wa_out[0] = 2 * ((p13 - p42)*rm[1][0] + (p23 + p41)*rm[1][1] - (p11 + p22)*rm[1][2]);
		wa_out[1] = 2 * (-(p22 + p33)*rm[2][0] + (p12 - p43)*rm[2][1] + (p13 + p42)*rm[2][2]);
		wa_out[2] = 2 * ((p12 + p43)*rm[0][0] - (p11 + p33)*rm[0][1] + (p23 - p41)*rm[0][2]);

		return wa_out;
	}
	auto s_wa2wq(const double *wa_in, const double *rq_in, double *wq_out) noexcept->double *
	{
		// 补充默认参数 //
		wa_in = wa_in ? wa_in : default_wa();
		rq_in = rq_in ? rq_in : default_rq();
		wq_out = wq_out ? wq_out : default_out();

		// 正式开始计算 //
		static const double S[4][3]{ { 1,-1,-1 },{ -1,1,-1 },{ -1,-1,1 },{ 1,1,1 } };
		static const double T[4][4]{ { 0,1,1,-1 },{ 1,0,1,-1 },{ 1,1,0,-1 },{ -1,-1,-1,0 } };
		static const int P[4][4]{ { -1,0,0,2 },{ 1,-1,1,0 },{ 2,2,-1,1 },{ 2,0,1,-1 } };
		static const int Q[4][4]{ { -1,1,2,1 },{ 0,-1,2,2 },{ 0,1,-1,0 },{ 1,2,0,-1 } };

		double rm[3][3], wm[3][3];

		s_rq2rm(rq_in, *rm);
		s_wa2wm(wa_in, *rm, *wm);

		int i = static_cast<int>(std::max_element(rq_in, rq_in + 4, [](double a, double b) {return std::abs(a) < std::abs(b); }) - rq_in);
		int jkl[3]{ (i + 1) % 4 ,(i + 2) % 4 ,(i + 3) % 4 };

		wq_out[i] = (S[i][0] * wm[0][0] + S[i][1] * wm[1][1] + S[i][2] * wm[2][2]) / 8 / rq_in[i];

		for (auto m : jkl) wq_out[m] = (wm[P[i][m]][Q[i][m]] + T[i][m] * wm[Q[i][m]][P[i][m]] - 4 * rq_in[m] * wq_out[i]) / 4 / rq_in[i];

		return wq_out;
	}
	auto s_wm2wa(const double *rm_in, const double *wm_in, double *wa_out, Size rm_ld, Size wm_ld) noexcept->double *
	{
		// 补充默认参数 //
		rm_in = rm_in ? rm_in : default_rm();
		wm_in = wm_in ? wm_in : default_wm();
		wa_out = wa_out ? wa_out : default_out();

		// 正式开始计算 //

		// dR = w x R
		// w x = dR * R' 

		wa_out[0] = wm_in[2 * wm_ld + 0] * rm_in[1 * rm_ld + 0] + wm_in[2 * wm_ld + 1] * rm_in[1 * rm_ld + 1] + wm_in[2 * wm_ld + 2] * rm_in[1 * rm_ld + 2];
		wa_out[1] = wm_in[0 * wm_ld + 0] * rm_in[2 * rm_ld + 0] + wm_in[0 * wm_ld + 1] * rm_in[2 * rm_ld + 1] + wm_in[0 * wm_ld + 2] * rm_in[2 * rm_ld + 2];
		wa_out[2] = wm_in[1 * wm_ld + 0] * rm_in[0 * rm_ld + 0] + wm_in[1 * wm_ld + 1] * rm_in[0 * rm_ld + 1] + wm_in[1 * wm_ld + 2] * rm_in[0 * rm_ld + 2];

		return wa_out;
	}
	auto s_wa2wm(const double *wa_in, const double *rm_in, double *wm_out, Size rm_ld, Size wm_ld) noexcept->double *
	{
		// 补充默认参数 //
		wa_in = wa_in ? wa_in : default_wa();
		rm_in = rm_in ? rm_in : default_rm();
		wm_out = wm_out ? wm_out : default_out();

		// 正式开始计算 //
		s_c3_n(3, wa_in, 1, rm_in, rm_ld, wm_out, wm_ld);

		return wm_out;
	}
	auto s_vp2vs(const double *pp_in, const double *vp_in, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		vp_in = vp_in ? vp_in : default_vp();
		pp_in = pp_in ? pp_in : default_pp();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		s_c3(pp_in, vs_out + 3, vs_out);
		s_va(3, vp_in, vs_out);

		return vs_out;
	}
	auto s_vs2vp(const double *vs_in, const double *pp_in, double *vp_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		pp_in = pp_in ? pp_in : default_pp();
		vp_out = vp_out ? vp_out : default_out();

		// 正式开始计算 //
		s_c3(vs_in + 3, pp_in, vp_out);
		s_va(3, vs_in, vp_out);

		return vp_out;
	}
	auto s_we2vs(const double *re_in, const double *we_in, double *vs_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		re_in = re_in ? re_in : default_re();
		we_in = we_in ? we_in : default_we();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		s_we2wa(re_in, we_in, vs_out + 3, eu_type_in);

		return vs_out;
	}
	auto s_vs2we(const double *vs_in, const double *re_in, double *we_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		re_in = re_in ? re_in : default_re();
		we_out = we_out ? we_out : default_out();

		// 正式开始计算 //
		s_wa2we(vs_in + 3, re_in, we_out, eu_type_in);

		return we_out;
	}
	auto s_wq2vs(const double *rq_in, const double *wq_in, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		rq_in = rq_in ? rq_in : default_rq();
		wq_in = wq_in ? wq_in : default_wq();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		s_wq2wa(rq_in, wq_in, vs_out + 3);

		return vs_out;
	}
	auto s_vs2wq(const double *vs_in, const double *rq_in, double *wq_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		rq_in = rq_in ? rq_in : default_rq();
		wq_out = wq_out ? wq_out : default_out();

		// 正式开始计算 //
		s_wa2wq(vs_in + 3, rq_in, wq_out);

		return wq_out;
	}
	auto s_wm2vs(const double *rm_in, const double *wm_in, double *vs_out, Size rm_ld, Size wm_ld) noexcept->double *
	{
		// 补充默认参数 //
		rm_in = rm_in ? rm_in : default_rm();
		wm_in = wm_in ? wm_in : default_wm();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		s_wm2wa(rm_in, wm_in, vs_out + 3, rm_ld, wm_ld);

		return vs_out;
	}
	auto s_vs2wm(const double *vs_in, const double *rm_in, double *wm_out, Size rm_ld, Size wm_ld) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		rm_in = rm_in ? rm_in : default_rm();
		wm_out = wm_out ? wm_out : default_out();

		// 正式开始计算 //
		s_wa2wm(vs_in + 3, rm_in, wm_out, rm_ld, wm_ld);

		return wm_out;
	}
	auto s_wa2vs(const double *wa_in, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		wa_in = wa_in ? wa_in : default_wa();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		std::copy(wa_in, wa_in + 3, vs_out + 3);

		return vs_out;
	}
	auto s_vs2wa(const double *vs_in, double *wa_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		wa_out = wa_out ? wa_out : default_out();

		// 正式开始计算 //
		std::copy(vs_in + 3, vs_in + 6, wa_out);

		return wa_out;
	}
	auto s_ve2vs(const double *pe_in, const double *ve_in, double *vs_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		pe_in = pe_in ? pe_in : default_pe();
		ve_in = ve_in ? ve_in : default_ve();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		s_we2vs(pe_in + 3, ve_in + 3, vs_out, eu_type_in);
		s_vp2vs(pe_in, ve_in, vs_out);

		return vs_out;
	}
	auto s_vs2ve(const double *vs_in, const double *pe_in, double *ve_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		pe_in = pe_in ? pe_in : default_pe();
		ve_out = ve_out ? ve_out : default_out();

		// 正式开始计算 //
		s_vs2we(vs_in, pe_in + 3, ve_out + 3, eu_type_in);
		s_vs2vp(vs_in, pe_in, ve_out);

		return ve_out;
	}
	auto s_vq2vs(const double *pq_in, const double *vq_in, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		pq_in = pq_in ? pq_in : default_pq();
		vq_in = vq_in ? vq_in : default_vq();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		s_wq2vs(pq_in + 3, vq_in + 3, vs_out);
		s_vp2vs(pq_in, vq_in, vs_out);

		return vs_out;
	}
	auto s_vs2vq(const double *vs_in, const double *pq_in, double *vq_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		pq_in = pq_in ? pq_in : default_pq();
		vq_out = vq_out ? vq_out : default_out();

		// 正式开始计算 //
		s_vs2wq(vs_in, pq_in + 3, vq_out + 3);
		s_vs2vp(vs_in, pq_in, vq_out);

		return vq_out;
	}
	auto s_vm2vs(const double *pm_in, const double *vm_in, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		vm_in = vm_in ? vm_in : default_vm();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		double pp[3]{ pm_in[3],pm_in[7],pm_in[11] };
		double vp[3]{ vm_in[3],vm_in[7],vm_in[11] };
		s_wm2vs(pm_in, vm_in, vs_out, 4, 4);
		s_vp2vs(pp, vp, vs_out);

		return vs_out;
	}
	auto s_vs2vm(const double *vs_in, const double *pm_in, double *vm_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		pm_in = pm_in ? pm_in : default_pm();
		vm_out = vm_out ? vm_out : default_out();

		// 正式开始计算 //
		double pp[3]{ pm_in[3],pm_in[7],pm_in[11] };
		double vp[3];
		s_vs2wm(vs_in, pm_in, vm_out, 4, 4);
		s_vs2vp(vs_in, pp, vp);

		vm_out[3] = vp[0];
		vm_out[7] = vp[1];
		vm_out[11] = vp[2];

		std::fill(vm_out + 12, vm_out + 16, 0);

		return vm_out;
	}
	auto s_va2vs(const double *pp_in, const double *va_in, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		pp_in = pp_in ? pp_in : default_pp();
		va_in = va_in ? va_in : default_va();
		vs_out = vs_out ? vs_out : default_out();

		// 正式开始计算 //
		s_wa2vs(va_in + 3, vs_out);
		s_vp2vs(pp_in, va_in, vs_out);

		return vs_out;
	}
	auto s_vs2va(const double *vs_in, const double *pp_in, double *va_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		pp_in = pp_in ? pp_in : default_pp();
		va_out = va_out ? va_out : default_out();

		// 正式开始计算 //
		s_vs2wa(vs_in, va_out + 3);
		s_vs2vp(vs_in, pp_in, va_out);

		return va_out;
	}

	auto s_xe2xa(const double *re_in, const double *we_in, const double *xe_in, double *xa_out, double *wa_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		re_in = re_in ? re_in : default_re();
		we_in = we_in ? we_in : default_we();
		xe_in = xe_in ? xe_in : default_xe();
		xa_out = xa_out ? xa_out : default_out();
		double wa_out_default[3];
		wa_out = wa_out ? wa_out : wa_out_default;


		// 正式开始计算 //
		const Size a = eu_type_in[0] - '1';
		const Size b = eu_type_in[1] - '1';
		const Size c = eu_type_in[2] - '1';
		const Size d = 3 - a - b;
		const Size e = 3 - b - c;

		const double s1 = std::sin(re_in[0]);
		const double c1 = std::cos(re_in[0]);
		const double s2 = std::sin(re_in[1]);
		const double c2 = std::cos(re_in[1]);

		const double ds1 = c1 * we_in[0];
		const double dc1 = -s1 * we_in[0];
		const double ds2 = c2 * we_in[1];
		const double dc2 = -s2 * we_in[1];

		const double Ab1 = c1;
		const double Ad1 = P()[d][b] * s1;
		const double Aa2 = c == a ? c2 : P()[a][d] * s2;
		const double Ab2 = c == a ? -s1 * s2 : P()[b][d] * s1*c2;
		const double Ad2 = c == a ? P()[d][a] * c1* s2 : c1 * c2;

		const double dAb1 = dc1;
		const double dAd1 = P()[d][b] * ds1;
		const double dAa2 = c == a ? dc2 : P()[a][d] * ds2;
		const double dAb2 = c == a ? -ds1 * s2 - s1 * ds2 : P()[b][d] * (ds1*c2 + s1 * dc2);
		const double dAd2 = c == a ? P()[d][a] * (dc1* s2 + c1 * ds2) : dc1 * c2 + c1 * dc2;

		wa_out[a] = we_in[0] + Aa2 * we_in[2];
		wa_out[b] = Ab1 * we_in[1] + Ab2 * we_in[2];
		wa_out[d] = Ad1 * we_in[1] + Ad2 * we_in[2];

		xa_out[a] = xe_in[0] + Aa2 * xe_in[2] + dAa2 * we_in[2];
		xa_out[b] = Ab1 * xe_in[1] + Ab2 * xe_in[2] + dAb1 * we_in[1] + dAb2 * we_in[2];
		xa_out[d] = Ad1 * xe_in[1] + Ad2 * xe_in[2] + dAd1 * we_in[1] + dAd2 * we_in[2];

		return xa_out;
	}
	auto s_xa2xe(const double *wa_in, const double *xa_in, const double *re_in, double *xe_out, double *we_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		wa_in = wa_in ? wa_in : default_wa();
		xa_in = xa_in ? xa_in : default_xa();
		re_in = re_in ? re_in : default_re();
		xe_out = xe_out ? xe_out : default_out();
		double we_out_default[3];
		we_out = we_out ? we_out : we_out_default;


		// 正式开始计算 //
		const Size a = eu_type_in[0] - '1';
		const Size b = eu_type_in[1] - '1';
		const Size c = eu_type_in[2] - '1';
		const Size d = 3 - a - b;
		const Size e = 3 - b - c;

		const double s1 = std::sin(re_in[0]);
		const double c1 = std::cos(re_in[0]);
		const double s2 = std::sin(re_in[1]);
		const double c2 = std::cos(re_in[1]);

		const double Ab1 = c1;
		const double Ad1 = P()[d][b] * s1;
		const double Aa2 = c == a ? c2 : P()[a][d] * s2;
		const double Ab2 = c == a ? -s1 * s2 : P()[b][d] * s1*c2;
		const double Ad2 = c == a ? P()[d][a] * c1* s2 : c1 * c2;

		const double M = (Ab1 * Ad2 - Ad1 * Ab2);
		const double N = (Ad2 * Ab1 - Ab2 * Ad1);

		we_out[1] = (wa_in[b] * Ad2 - wa_in[d] * Ab2) / M;
		we_out[2] = (wa_in[d] * Ab1 - wa_in[b] * Ad1) / N;
		we_out[0] = (wa_in[a] - Aa2 * we_out[2]);

		const double ds1 = c1 * we_out[0];
		const double dc1 = -s1 * we_out[0];
		const double ds2 = c2 * we_out[1];
		const double dc2 = -s2 * we_out[1];

		const double dAb1 = dc1;
		const double dAd1 = P()[d][b] * ds1;
		const double dAa2 = c == a ? dc2 : P()[a][d] * ds2;
		const double dAb2 = c == a ? -ds1 * s2 - s1 * ds2 : P()[b][d] * (ds1*c2 + s1 * dc2);
		const double dAd2 = c == a ? P()[d][a] * (dc1* s2 + c1 * ds2) : dc1 * c2 + c1 * dc2;

		const double ba = xa_in[a] - dAa2 * we_out[2];
		const double bb = xa_in[b] - dAb1 * we_out[1] - dAb2 * we_out[2];
		const double bd = xa_in[d] - dAd1 * we_out[1] - dAd2 * we_out[2];

		xe_out[1] = (bb * Ad2 - bd * Ab2) / M;
		xe_out[2] = (bd * Ab1 - bb * Ad1) / N;
		xe_out[0] = (ba - Aa2 * xe_out[2]);

		return xe_out;
	}
	auto s_xq2xa(const double *rq_in, const double *wq_in, const double *xq_in, double *xa_out, double *wa_out) noexcept->double *
	{
		// 补充默认参数 //
		rq_in = rq_in ? rq_in : default_rq();
		wq_in = wq_in ? wq_in : default_wq();
		xq_in = xq_in ? xq_in : default_xq();
		double wa_out_default[3];
		wa_out = wa_out ? wa_out : wa_out_default;
		xa_out = xa_out ? xa_out : default_out();

		// 正式开始计算 //
		double p11 = 2 * wq_in[0] * rq_in[0];
		double p22 = 2 * wq_in[1] * rq_in[1];
		double p33 = 2 * wq_in[2] * rq_in[2];
		double p12 = wq_in[0] * rq_in[1] + rq_in[0] * wq_in[1];
		double p13 = wq_in[0] * rq_in[2] + rq_in[0] * wq_in[2];
		double p23 = wq_in[1] * rq_in[2] + rq_in[1] * wq_in[2];
		double p41 = wq_in[3] * rq_in[0] + rq_in[3] * wq_in[0];
		double p42 = wq_in[3] * rq_in[1] + rq_in[3] * wq_in[1];
		double p43 = wq_in[3] * rq_in[2] + rq_in[3] * wq_in[2];

		double rm[3][3], wm[3][3];
		s_rq2rm(rq_in, *rm);

		wa_out[0] = 2 * ((p13 - p42)*rm[1][0] + (p23 + p41)*rm[1][1] - (p11 + p22)*rm[1][2]);
		wa_out[1] = 2 * (-(p22 + p33)*rm[2][0] + (p12 - p43)*rm[2][1] + (p13 + p42)*rm[2][2]);
		wa_out[2] = 2 * ((p12 + p43)*rm[0][0] - (p11 + p33)*rm[0][1] + (p23 - p41)*rm[0][2]);

		s_wa2wm(wa_out, *rm, *wm);

		double t11 = 2 * (xq_in[0] * rq_in[0] + wq_in[0] * wq_in[0]);
		double t22 = 2 * (xq_in[1] * rq_in[1] + wq_in[1] * wq_in[1]);
		double t33 = 2 * (xq_in[2] * rq_in[2] + wq_in[2] * wq_in[2]);
		double t12 = xq_in[0] * rq_in[1] + 2 * wq_in[0] * wq_in[1] + rq_in[0] * xq_in[1];
		double t13 = xq_in[0] * rq_in[2] + 2 * wq_in[0] * wq_in[2] + rq_in[0] * xq_in[2];
		double t23 = xq_in[1] * rq_in[2] + 2 * wq_in[1] * wq_in[2] + rq_in[1] * xq_in[2];
		double t41 = xq_in[3] * rq_in[0] + 2 * wq_in[3] * wq_in[0] + rq_in[3] * xq_in[0];
		double t42 = xq_in[3] * rq_in[1] + 2 * wq_in[3] * wq_in[1] + rq_in[3] * xq_in[1];
		double t43 = xq_in[3] * rq_in[2] + 2 * wq_in[3] * wq_in[2] + rq_in[3] * xq_in[2];

		xa_out[0] = 2 * ((t13 - t42)*rm[1][0] + (t23 + t41)*rm[1][1] - (t11 + t22)*rm[1][2]) + 2 * ((p13 - p42)*wm[1][0] + (p23 + p41)*wm[1][1] - (p11 + p22)*wm[1][2]);
		xa_out[1] = 2 * (-(t22 + t33)*rm[2][0] + (t12 - t43)*rm[2][1] + (t13 + t42)*rm[2][2]) + 2 * (-(p22 + p33)*wm[2][0] + (p12 - p43)*wm[2][1] + (p13 + p42)*wm[2][2]);
		xa_out[2] = 2 * ((t12 + t43)*rm[0][0] - (t11 + t33)*rm[0][1] + (t23 - t41)*rm[0][2]) + 2 * ((p12 + p43)*wm[0][0] - (p11 + p33)*wm[0][1] + (p23 - p41)*wm[0][2]);

		return xa_out;
	}
	auto s_xa2xq(const double *wa_in, const double *xa_in, const double *rq_in, double *xq_out, double *wq_out) noexcept->double *
	{
		// 补充默认参数 //
		wa_in = wa_in ? wa_in : default_wa();
		xa_in = xa_in ? xa_in : default_xa();
		rq_in = rq_in ? rq_in : default_rq();
		double wq_out_default[4];
		xq_out = xq_out ? xq_out : default_out();
		wq_out = wq_out ? wq_out : wq_out_default;

		// 正式开始计算 //
		static const double S[4][3]{ { 1,-1,-1 },{ -1,1,-1 },{ -1,-1,1 },{ 1,1,1 } };
		static const double T[4][4]{ { 0,1,1,-1 },{ 1,0,1,-1 },{ 1,1,0,-1 },{ -1,-1,-1,0 } };
		static const int P[4][4]{ { -1,0,0,2 },{ 1,-1,1,0 },{ 2,2,-1,1 },{ 2,0,1,-1 } };
		static const int Q[4][4]{ { -1,1,2,1 },{ 0,-1,2,2 },{ 0,1,-1,0 },{ 1,2,0,-1 } };

		double rm[3][3], wm[3][3], xm[3][3];

		s_rq2rm(rq_in, *rm);
		s_xa2xm(wa_in, xa_in, *rm, *xm, *wm);

		int i = static_cast<int>(std::max_element(rq_in, rq_in + 4, [](double a, double b) {return std::abs(a) < std::abs(b); }) - rq_in);
		int jkl[3]{ (i + 1) % 4 ,(i + 2) % 4 ,(i + 3) % 4 };

		wq_out[i] = (S[i][0] * wm[0][0] + S[i][1] * wm[1][1] + S[i][2] * wm[2][2]) / 8 / rq_in[i];
		xq_out[i] = (S[i][0] * xm[0][0] + S[i][1] * xm[1][1] + S[i][2] * xm[2][2] - 8 * wq_out[i] * wq_out[i]) / 8 / rq_in[i];

		for (auto m : jkl)
		{
			wq_out[m] = (wm[P[i][m]][Q[i][m]] + T[i][m] * wm[Q[i][m]][P[i][m]] - 4 * rq_in[m] * wq_out[i]) / 4 / rq_in[i];
			xq_out[m] = (xm[P[i][m]][Q[i][m]] + T[i][m] * xm[Q[i][m]][P[i][m]] - 8 * wq_out[m] * wq_out[i] - 4 * rq_in[m] * xq_out[i]) / 4 / rq_in[i];
		}

		return xq_out;
	}
	auto s_xm2xa(const double *rm_in, const double *wm_in, const double *xm_in, double *xa_out, double *wa_out, Size rm_ld, Size wm_ld, Size xm_ld) noexcept->double *
	{
		// 补充默认参数 //
		rm_in = rm_in ? rm_in : default_rm();
		wm_in = wm_in ? wm_in : default_wm();
		xm_in = xm_in ? xm_in : default_xm();
		double wa_out_default[3];
		wa_out = wa_out ? wa_out : wa_out_default;
		xa_out = xa_out ? xa_out : default_out();

		// 正式开始计算 //

		// ddR = x x R + w x dR
		// x x = (ddR - w x dR) * R' 

		s_wm2wa(rm_in, wm_in, wa_out, rm_ld, wm_ld);

		double tem[9];
		std::copy(xm_in + 0 * xm_ld, xm_in + 0 * xm_ld + 3, tem + 0);
		std::copy(xm_in + 1 * xm_ld, xm_in + 1 * xm_ld + 3, tem + 3);
		std::copy(xm_in + 2 * xm_ld, xm_in + 2 * xm_ld + 3, tem + 6);

		s_c3s_n(3, wa_out, 1, wm_in, wm_ld, tem, 3);

		xa_out[0] = tem[2 * 3 + 0] * rm_in[1 * rm_ld + 0] + tem[2 * 3 + 1] * rm_in[1 * rm_ld + 1] + tem[2 * 3 + 2] * rm_in[1 * rm_ld + 2];
		xa_out[1] = tem[0 * 3 + 0] * rm_in[2 * rm_ld + 0] + tem[0 * 3 + 1] * rm_in[2 * rm_ld + 1] + tem[0 * 3 + 2] * rm_in[2 * rm_ld + 2];
		xa_out[2] = tem[1 * 3 + 0] * rm_in[0 * rm_ld + 0] + tem[1 * 3 + 1] * rm_in[0 * rm_ld + 1] + tem[1 * 3 + 2] * rm_in[0 * rm_ld + 2];

		return xa_out;
	}
	auto s_xa2xm(const double *wa_in, const double *xa_in, const double *rm_in, double *xm_out, double *wm_out, Size rm_ld, Size wm_ld, Size xm_ld) noexcept->double *
	{
		// 补充默认参数 //
		wa_in = wa_in ? wa_in : default_wa();
		xa_in = xa_in ? xa_in : default_xa();
		rm_in = rm_in ? rm_in : default_rm();
		double wm_out_default[9];
		wm_out = wm_out ? wm_out : wm_out_default;
		xm_out = xm_out ? xm_out : default_out();

		// 正式开始计算 //

		// ddR = x x R + w x dR
		// w x = dR * R' 

		s_wa2wm(wa_in, rm_in, wm_out, rm_ld, wm_ld);

		s_c3_n(3, wa_in, 1, wm_out, wm_ld, xm_out, xm_ld);
		s_c3a_n(3, xa_in, 1, rm_in, rm_ld, xm_out, xm_ld);

		return xm_out;
	}
	auto s_ap2as(const double *pp_in, const double *vp_in, const double *ap_in, double *as_out, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		pp_in = pp_in ? pp_in : default_pp();
		vp_in = vp_in ? vp_in : default_vp();
		ap_in = ap_in ? ap_in : default_ap();
		double vs_out_default[6]{ 0,0,0,0,0,0 };
		as_out = as_out ? as_out : default_out();
		vs_out = vs_out ? vs_out : vs_out_default;

		// 正式开始计算 //
		s_vp2vs(pp_in, vp_in, vs_out);

		std::copy(ap_in, ap_in + 3, as_out);
		s_c3s(vs_out + 3, vp_in, as_out);
		s_c3s(as_out + 3, pp_in, as_out);

		return as_out;
	}
	auto s_as2ap(const double *vs_in, const double *as_in, const double *pp_in, double *ap_out, double *vp_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		as_in = as_in ? as_in : default_as();
		pp_in = pp_in ? pp_in : default_pp();
		double vp_out_default[3];
		ap_out = ap_out ? ap_out : default_out();
		vp_out = vp_out ? vp_out : vp_out_default;

		// 正式开始计算 //
		s_vs2vp(vs_in, pp_in, vp_out);

		std::copy(as_in, as_in + 3, ap_out);
		s_c3a(vs_in + 3, vp_out, ap_out);
		s_c3a(as_in + 3, pp_in, ap_out);

		return ap_out;
	}
	auto s_xe2as(const double *re_in, const double *we_in, const double *xe_in, double *as_out, double *vs_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		re_in = re_in ? re_in : default_re();
		we_in = we_in ? we_in : default_we();
		xe_in = xe_in ? xe_in : default_xe();
		double vs_out_default[6]{ 0,0,0,0,0,0 };
		vs_out = vs_out ? vs_out : vs_out_default;
		as_out = as_out ? as_out : default_out();

		// 正式开始计算 //
		s_xe2xa(re_in, we_in, xe_in, as_out + 3, vs_out + 3, eu_type_in);

		return as_out;
	}
	auto s_as2xe(const double *vs_in, const double *as_in, const double *re_in, double *xe_out, double *we_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		as_in = as_in ? as_in : default_as();
		re_in = re_in ? re_in : default_re();
		double we_out_default[3];
		we_out = we_out ? we_out : we_out_default;
		xe_out = xe_out ? xe_out : default_out();

		// 正式开始计算 //
		s_xa2xe(vs_in + 3, as_in + 3, re_in, xe_out, we_out, eu_type_in);

		return xe_out;
	}
	auto s_xq2as(const double *rq_in, const double *wq_in, const double *xq_in, double *as_out, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		rq_in = rq_in ? rq_in : default_rq();
		wq_in = wq_in ? wq_in : default_wq();
		xq_in = xq_in ? xq_in : default_xq();
		double vs_out_default[6]{ 0,0,0,0,0,0 };
		vs_out = vs_out ? vs_out : vs_out_default;
		as_out = as_out ? as_out : default_out();

		// 正式开始计算 //
		s_xq2xa(rq_in, wq_in, xq_in, as_out + 3, vs_out + 3);

		return as_out;
	}
	auto s_as2xq(const double *vs_in, const double *as_in, const double *rq_in, double *xq_out, double *wq_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		as_in = as_in ? as_in : default_as();
		rq_in = rq_in ? rq_in : default_rq();
		double wq_out_default[4];
		wq_out = wq_out ? wq_out : wq_out_default;
		xq_out = xq_out ? xq_out : default_out();

		// 正式开始计算 //
		s_xa2xq(vs_in + 3, as_in + 3, rq_in, xq_out, wq_out);

		return xq_out;
	}
	auto s_xm2as(const double *rm_in, const double *wm_in, const double *xm_in, double *as_out, double *vs_out, Size rm_ld, Size wm_ld, Size xm_ld) noexcept->double *
	{
		// 补充默认参数 //
		rm_in = rm_in ? rm_in : default_rm();
		wm_in = wm_in ? wm_in : default_wm();
		xm_in = xm_in ? xm_in : default_xm();
		double vs_out_default[6]{ 0,0,0,0,0,0 };
		vs_out = vs_out ? vs_out : vs_out_default;
		as_out = as_out ? as_out : default_out();

		// 正式开始计算 //
		s_xm2xa(rm_in, wm_in, xm_in, as_out + 3, vs_out + 3, rm_ld, wm_ld, xm_ld);

		return as_out;
	}
	auto s_as2xm(const double *vs_in, const double *as_in, const double *rm_in, double *xm_out, double *wm_out, Size rm_ld, Size wm_ld, Size xm_ld) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		as_in = as_in ? as_in : default_as();
		rm_in = rm_in ? rm_in : default_rm();
		double wm_out_default[9];
		wm_out = wm_out ? wm_out : wm_out_default;
		xm_out = xm_out ? xm_out : default_out();

		// 正式开始计算 //
		s_xa2xm(vs_in + 3, as_in + 3, rm_in, xm_out, wm_out, rm_ld, wm_ld, xm_ld);

		return xm_out;
	}
	auto s_xa2as(const double *xa_in, double *as_out) noexcept->double *
	{
		// 补充默认参数 //
		xa_in = xa_in ? xa_in : default_xa();
		as_out = as_out ? as_out : default_out();

		// 正式开始计算 //
		std::copy(xa_in, xa_in + 3, as_out + 3);

		return as_out;
	}
	auto s_as2xa(const double *as_in, double *xa_out) noexcept->double *
	{
		// 补充默认参数 //
		as_in = as_in ? as_in : default_as();
		xa_out = xa_out ? xa_out : default_out();

		// 正式开始计算 //
		std::copy(as_in + 3, as_in + 6, xa_out);

		return xa_out;
	}
	auto s_ae2as(const double *pe_in, const double *ve_in, const double *ae_in, double *as_out, double *vs_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		pe_in = pe_in ? pe_in : default_pe();
		ve_in = ve_in ? ve_in : default_ve();
		ae_in = ae_in ? ae_in : default_ae();
		double vs_out_default[6];
		vs_out = vs_out ? vs_out : vs_out_default;
		as_out = as_out ? as_out : default_out();

		// 正式开始计算 //
		s_xe2as(pe_in + 3, ve_in + 3, ae_in + 3, as_out, vs_out, eu_type_in);
		s_ap2as(pe_in, ve_in, ae_in, as_out, vs_out);

		return as_out;
	}
	auto s_as2ae(const double *vs_in, const double *as_in, const double *pe_in, double *ae_out, double *ve_out, const char *eu_type_in) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		as_in = as_in ? as_in : default_as();
		pe_in = pe_in ? pe_in : default_pe();
		double ve_out_default[6];
		ve_out = ve_out ? ve_out : ve_out_default;
		ae_out = ae_out ? ae_out : default_out();

		// 正式开始计算 //
		s_as2xe(vs_in, as_in, pe_in + 3, ae_out + 3, ve_out + 3, eu_type_in);
		s_as2ap(vs_in, as_in, pe_in, ae_out, ve_out);

		return ae_out;
	}
	auto s_aq2as(const double *pq_in, const double *vq_in, const double *aq_in, double *as_out, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		pq_in = pq_in ? pq_in : default_pq();
		vq_in = vq_in ? vq_in : default_vq();
		aq_in = aq_in ? aq_in : default_aq();
		double vs_out_default[6];
		vs_out = vs_out ? vs_out : vs_out_default;
		as_out = as_out ? as_out : default_out();

		// 正式开始计算 //
		s_xq2as(pq_in + 3, vq_in + 3, aq_in + 3, as_out, vs_out);
		s_ap2as(pq_in, vq_in, aq_in, as_out, vs_out);

		return as_out;

	}
	auto s_as2aq(const double *vs_in, const double *as_in, const double *pq_in, double *aq_out, double *vq_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		as_in = as_in ? as_in : default_as();
		pq_in = pq_in ? pq_in : default_pq();
		double vq_out_default[7];
		vq_out = vq_out ? vq_out : vq_out_default;
		aq_out = aq_out ? aq_out : default_out();

		// 正式开始计算 //
		s_as2xq(vs_in, as_in, pq_in + 3, aq_out + 3, vq_out + 3);
		s_as2ap(vs_in, as_in, pq_in, aq_out, vq_out);

		return aq_out;
	}
	auto s_am2as(const double *pm_in, const double *vm_in, const double *am_in, double *as_out, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		pm_in = pm_in ? pm_in : default_pm();
		vm_in = vm_in ? vm_in : default_vm();
		am_in = am_in ? am_in : default_am();
		double vs_out_default[6];
		vs_out = vs_out ? vs_out : vs_out_default;
		as_out = as_out ? as_out : default_out();

		// 正式开始计算 //
		double pp[3]{ pm_in[3],pm_in[7],pm_in[11] };
		double vp[3]{ vm_in[3],vm_in[7],vm_in[11] };
		double ap[3]{ am_in[3],am_in[7],am_in[11] };

		s_xm2as(pm_in, vm_in, am_in, as_out, vs_out, 4, 4, 4);
		s_ap2as(pp, vp, ap, as_out, vs_out);

		return as_out;
	}
	auto s_as2am(const double *vs_in, const double *as_in, const double *pm_in, double *am_out, double *vm_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		as_in = as_in ? as_in : default_as();
		pm_in = pm_in ? pm_in : default_pm();
		double vm_out_default[16];
		vm_out = vm_out ? vm_out : vm_out_default;
		am_out = am_out ? am_out : default_out();

		// 正式开始计算 //
		double pp[3]{ pm_in[3], pm_in[7], pm_in[11] };
		double vp[3], ap[3];

		s_as2ap(vs_in, as_in, pp, ap, vp);
		s_as2xm(vs_in, as_in, pm_in, am_out, vm_out, 4, 4, 4);

		vm_out[3] = vp[0];
		vm_out[7] = vp[1];
		vm_out[11] = vp[2];
		std::fill(vm_out + 12, vm_out + 16, 0);

		am_out[3] = ap[0];
		am_out[7] = ap[1];
		am_out[11] = ap[2];
		std::fill(am_out + 12, am_out + 16, 0);

		return am_out;
	}
	auto s_aa2as(const double *pp_in, const double *va_in, const double *aa_in, double *as_out, double *vs_out) noexcept->double *
	{
		// 补充默认参数 //
		pp_in = pp_in ? pp_in : default_pp();
		va_in = va_in ? va_in : default_va();
		aa_in = aa_in ? aa_in : default_aa();
		double vs_out_default[6];
		vs_out = vs_out ? vs_out : vs_out_default;
		as_out = as_out ? as_out : default_out();

		// 正式开始计算 //
		s_wa2vs(va_in + 3, vs_out);
		s_xa2as(aa_in + 3, as_out);
		s_ap2as(pp_in, va_in, aa_in, as_out, vs_out);

		return as_out;
	}
	auto s_as2aa(const double *vs_in, const double *as_in, const double *pp_in, double *aa_out, double *va_out) noexcept->double *
	{
		// 补充默认参数 //
		vs_in = vs_in ? vs_in : default_vs();
		as_in = as_in ? as_in : default_as();
		pp_in = pp_in ? pp_in : default_pp();
		double va_out_default[6];
		va_out = va_out ? va_out : va_out_default;
		aa_out = aa_out ? aa_out : default_out();

		// 正式开始计算 //
		s_vs2wa(vs_in, va_out + 3);
		s_as2xa(as_in, aa_out + 3);
		s_as2ap(vs_in, as_in, pp_in, aa_out, va_out);

		return aa_out;
	}

	auto s_pq2pe(const double *pq_in, double *pe_out, const char *eu_type) noexcept->double *
	{
		// 补充默认参数 //
		pq_in = pq_in ? pq_in : default_pq();
		pe_out = pe_out ? pe_out : default_out();

		// 正式开始计算 //

		double pm[16];
		s_pq2pm(pq_in, pm);
		s_pm2pe(pm, pe_out, eu_type);

		return pe_out;
	}
	auto s_pe2pq(const double *pe_in, double *pq_out, const char *eu_type) noexcept->double *
	{
		// 补充默认参数 //
		pe_in = pe_in ? pe_in : default_pe();
		pq_out = pq_out ? pq_out : default_out();

		// 正式开始计算 //
		double pm[16];
		s_pe2pm(pe_in, pm, eu_type);
		s_pm2pq(pm, pq_out);

		return pq_out;
	}
	auto s_iv2im(const double * iv_in, double *im_out) noexcept->double *
	{
		// 补充默认参数 //
		iv_in = iv_in ? iv_in : default_iv();
		im_out = im_out ? im_out : default_out();

		// 正式开始计算 //

		std::fill(im_out, im_out + 36, 0);

		im_out[0] = iv_in[0];
		im_out[7] = iv_in[0];
		im_out[14] = iv_in[0];

		im_out[4] = iv_in[3];
		im_out[5] = -iv_in[2];
		im_out[9] = -iv_in[3];
		im_out[11] = iv_in[1];
		im_out[15] = iv_in[2];
		im_out[16] = -iv_in[1];

		im_out[19] = -iv_in[3];
		im_out[20] = iv_in[2];
		im_out[24] = iv_in[3];
		im_out[26] = -iv_in[1];
		im_out[30] = -iv_in[2];
		im_out[31] = iv_in[1];

		im_out[21] = iv_in[4];
		im_out[22] = iv_in[7];
		im_out[23] = iv_in[8];
		im_out[27] = iv_in[7];
		im_out[28] = iv_in[5];
		im_out[29] = iv_in[9];
		im_out[33] = iv_in[8];
		im_out[34] = iv_in[9];
		im_out[35] = iv_in[6];

		return im_out;
	}
	auto s_im2iv(const double * im_in, double *iv_out) noexcept->double *
	{
		// 补充默认参数 //
		im_in = im_in ? im_in : default_im();
		iv_out = iv_out ? iv_out : default_out();

		// 正式开始计算 //
		iv_out[0] = im_in[0];
		iv_out[1] = im_in[11];
		iv_out[2] = im_in[15];
		iv_out[3] = im_in[4];
		iv_out[4] = im_in[21];
		iv_out[5] = im_in[28];
		iv_out[6] = im_in[35];
		iv_out[7] = im_in[22];
		iv_out[8] = im_in[23];
		iv_out[9] = im_in[29];

		return iv_out;
	}
	auto s_i32im(const double mass_in, const double * i3_in, const double *pm_in, double *is_out) noexcept->double *
	{
		// 补充默认参数 //
		i3_in = i3_in ? i3_in : default_i3();
		pm_in = pm_in ? pm_in : default_pm();
		is_out = is_out ? is_out : default_out();

		// 正式开始计算 //
		double tem[6][6], tmf[6][6];

		is_out[0] = mass_in;
		is_out[7] = mass_in;
		is_out[14] = mass_in;

		is_out[21] = i3_in[0];
		is_out[22] = i3_in[1];
		is_out[23] = i3_in[2];
		is_out[27] = i3_in[3];
		is_out[28] = i3_in[4];
		is_out[29] = i3_in[5];
		is_out[33] = i3_in[6];
		is_out[34] = i3_in[7];
		is_out[35] = i3_in[8];

		s_tmf(pm_in, *tmf);
		s_mm(6, 6, 6, *tmf, 6, is_out, 6, *tem, 6);
		s_mm(6, 6, 6, *tem, 6, *tmf, 6, is_out, 6);

		return is_out;
	}

	auto s_pp2pp(const double *relative_pm, const double *from_pp, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_pp = from_pp ? from_pp : default_pp();
		to_pp = to_pp ? to_pp : default_out();

		// 正式开始计算 //
		to_pp[0] = relative_pm[0] * from_pp[0] + relative_pm[1] * from_pp[1] + relative_pm[2] * from_pp[2] + relative_pm[3];
		to_pp[1] = relative_pm[4] * from_pp[0] + relative_pm[5] * from_pp[1] + relative_pm[6] * from_pp[2] + relative_pm[7];
		to_pp[2] = relative_pm[8] * from_pp[0] + relative_pm[9] * from_pp[1] + relative_pm[10] * from_pp[2] + relative_pm[11];

		return to_pp;
	}
	auto s_inv_pp2pp(const double *inv_relative_pm, const double *from_pp, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_pp = from_pp ? from_pp : default_pp();
		to_pp = to_pp ? to_pp : default_out();

		// 正式开始计算 //
		double tem[3]{ from_pp[0] - inv_relative_pm[3] ,from_pp[1] - inv_relative_pm[7] ,from_pp[2] - inv_relative_pm[11] };

		to_pp[0] = inv_relative_pm[0] * tem[0] + inv_relative_pm[4] * tem[1] + inv_relative_pm[8] * tem[2];
		to_pp[1] = inv_relative_pm[1] * tem[0] + inv_relative_pm[5] * tem[1] + inv_relative_pm[9] * tem[2];
		to_pp[2] = inv_relative_pm[2] * tem[0] + inv_relative_pm[6] * tem[1] + inv_relative_pm[10] * tem[2];

		return to_pp;
	}
	auto s_re2re(const double *relative_pm, const double *from_re, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_re = from_re ? from_re : default_re();
		to_re = to_re ? to_re : default_out();

		// 正式开始计算 //
		double from_rm[3][3], to_rm[3][3];
		s_re2rm(from_re, *from_rm, from_re_type);
		s_rm2rm(relative_pm, *from_rm, *to_rm);
		s_rm2re(*to_rm, to_re, to_re_type);

		return to_re;
	}
	auto s_inv_re2re(const double *inv_relative_pm, const double *from_re, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_re = from_re ? from_re : default_re();
		to_re = to_re ? to_re : default_out();

		// 正式开始计算 //
		double from_rm[3][3], to_rm[3][3];
		s_re2rm(from_re, *from_rm, from_re_type);
		s_inv_rm2rm(inv_relative_pm, *from_rm, *to_rm);
		s_rm2re(*to_rm, to_re, to_re_type);

		return to_re;
	}
	auto s_rq2rq(const double *relative_pm, const double *from_rq, double *to_rq) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_rq = from_rq ? from_rq : default_rq();
		to_rq = to_rq ? to_rq : default_out();

		// 正式开始计算 //
		double from_rm[3][3], to_rm[3][3];
		s_rq2rm(from_rq, *from_rm);
		s_rm2rm(relative_pm, *from_rm, *to_rm);
		s_rm2rq(*to_rm, to_rq);

		return to_rq;
	}
	auto s_inv_rq2rq(const double *inv_relative_pm, const double *from_rq, double *to_rq) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_rq = from_rq ? from_rq : default_rq();
		to_rq = to_rq ? to_rq : default_out();

		// 正式开始计算 //
		double from_rm[3][3], to_rm[3][3];
		s_rq2rm(from_rq, *from_rm);
		s_inv_rm2rm(inv_relative_pm, *from_rm, *to_rm);
		s_rm2rq(*to_rm, to_rq);

		return to_rq;
	}
	auto s_rm2rm(const double *relative_pm, const double *from_rm, double *to_rm, Size from_rm_ld, Size to_rm_ld) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_rm = from_rm ? from_rm : default_rm();
		to_rm = to_rm ? to_rm : default_out();

		// 正式开始计算 //
		s_mm(3, 3, 3, relative_pm, 4, from_rm, from_rm_ld, to_rm, to_rm_ld);

		return to_rm;
	}
	auto s_inv_rm2rm(const double *inv_relative_pm, const double *from_rm, double *to_rm, Size from_rm_ld, Size to_rm_ld) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_rm = from_rm ? from_rm : default_rm();
		to_rm = to_rm ? to_rm : default_out();

		// 正式开始计算 //
		s_mm(3, 3, 3, inv_relative_pm, ColMajor{ 4 }, from_rm, from_rm_ld, to_rm, to_rm_ld);

		return to_rm;
	}
	auto s_pe2pe(const double *relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type, const char *to_pe_type) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_pe = from_pe ? from_pe : default_pe();
		to_pe = to_pe ? to_pe : default_out();

		// 正式开始计算 //
		double from_pm[4][4], to_pm[4][4];
		s_pe2pm(from_pe, *from_pm, from_pe_type);
		s_pm2pm(relative_pm, *from_pm, *to_pm);
		s_pm2pe(*to_pm, to_pe, to_pe_type);

		return to_pe;
	}
	auto s_inv_pe2pe(const double *inv_relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type, const char *to_pe_type) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_pe = from_pe ? from_pe : default_pe();
		to_pe = to_pe ? to_pe : default_out();

		// 正式开始计算 //
		double from_pm[4][4], to_pm[4][4];
		s_pe2pm(from_pe, *from_pm, from_pe_type);
		s_inv_pm2pm(inv_relative_pm, *from_pm, *to_pm);
		s_pm2pe(*to_pm, to_pe, to_pe_type);

		return to_pe;
	}
	auto s_pq2pq(const double *relative_pm, const double *from_pq, double *to_pq) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_pq = from_pq ? from_pq : default_pq();
		to_pq = to_pq ? to_pq : default_out();

		// 正式开始计算 //
		double from_pm[4][4], to_pm[4][4];
		s_pq2pm(from_pq, *from_pm);
		s_pm2pm(relative_pm, *from_pm, *to_pm);
		s_pm2pq(*to_pm, to_pq);

		return to_pq;
	}
	auto s_inv_pq2pq(const double *inv_relative_pm, const double *from_pq, double *to_pq) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_pq = from_pq ? from_pq : default_pq();
		to_pq = to_pq ? to_pq : default_out();

		// 正式开始计算 //
		double from_pm[4][4], to_pm[4][4];
		s_pq2pm(from_pq, *from_pm);
		s_inv_pm2pm(inv_relative_pm, *from_pm, *to_pm);
		s_pm2pq(*to_pm, to_pq);

		return to_pq;
	}
	auto s_pm2pm(const double *relative_pm, const double *from_pm, double *to_pm) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_pm = from_pm ? from_pm : default_pm();
		to_pm = to_pm ? to_pm : default_out();

		// 正式开始计算 //
		s_mm(3, 4, 3, relative_pm, 4, from_pm, 4, to_pm, 4);

		to_pm[3] += relative_pm[3];
		to_pm[7] += relative_pm[7];
		to_pm[11] += relative_pm[11];

		to_pm[12] = 0;
		to_pm[13] = 0;
		to_pm[14] = 0;
		to_pm[15] = 1;

		return to_pm;
	}
	auto s_inv_pm2pm(const double *inv_relative_pm, const double *from_pm, double *to_pm) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_pm = from_pm ? from_pm : default_pm();
		to_pm = to_pm ? to_pm : default_out();

		// 正式开始计算 //
		s_mm(3, 4, 3, inv_relative_pm, ColMajor{ 4 }, from_pm, 4, to_pm, 4);

		to_pm[3] += -inv_relative_pm[0] * inv_relative_pm[3] - inv_relative_pm[4] * inv_relative_pm[7] - inv_relative_pm[8] * inv_relative_pm[11];
		to_pm[7] += -inv_relative_pm[1] * inv_relative_pm[3] - inv_relative_pm[5] * inv_relative_pm[7] - inv_relative_pm[9] * inv_relative_pm[11];
		to_pm[11] += -inv_relative_pm[2] * inv_relative_pm[3] - inv_relative_pm[6] * inv_relative_pm[7] - inv_relative_pm[10] * inv_relative_pm[11];

		to_pm[12] = 0;
		to_pm[13] = 0;
		to_pm[14] = 0;
		to_pm[15] = 1;

		return to_pm;
	}

	auto s_vp2vp(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_pp = from_pp ? from_pp : default_pp();
		from_vp = from_vp ? from_vp : default_pp();
		double default_to_pp[3];
		to_vp = to_vp ? to_vp : default_out();
		to_pp = to_pp ? to_pp : default_to_pp;

		// 正式开始计算 //
		s_pp2pp(relative_pm, from_pp, to_pp);
		s_c3(relative_vs + 3, to_pp, to_vp);
		s_mma(3, 1, 3, relative_pm, 4, from_vp, 1, to_vp, 1);
		s_va(3, relative_vs, to_vp);

		return to_vp;
	}
	auto s_inv_vp2vp(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_pp = from_pp ? from_pp : default_pp();
		from_vp = from_vp ? from_vp : default_vp();
		double default_to_pp[3];
		to_vp = to_vp ? to_vp : default_out();
		to_pp = to_pp ? to_pp : default_to_pp;

		// 正式计算开始 //
		s_inv_pp2pp(inv_relative_pm, from_pp, to_pp);

		double tem[3];
		std::copy_n(from_vp, 3, tem);
		s_c3s(inv_relative_vs + 3, from_pp, tem);
		s_vs(3, inv_relative_vs, tem);
		s_mm(3, 1, 3, inv_relative_pm, ColMajor{ 4 }, tem, 1, to_vp, 1);

		return to_vp;
	}
	auto s_we2we(const double *relative_pm, const double *relative_vs, const double *from_re, const double *from_we, double *to_we, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_re = from_re ? from_re : default_re();
		from_we = from_we ? from_we : default_we();
		double default_to_re[3];
		to_we = to_we ? to_we : default_out();
		to_re = to_re ? to_re : default_to_re;

		// 正式开始计算 //
		s_re2re(relative_pm, from_re, to_re, from_re_type, to_re_type);

		double from_wa[3], to_wa[3];
		s_we2wa(from_re, from_we, from_wa, from_re_type);
		s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);
		s_wa2we(to_wa, to_re, to_we, to_re_type);

		return to_we;
	}
	auto s_inv_we2we(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_re, const double *from_we, double *to_we, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_re = from_re ? from_re : default_re();
		from_we = from_we ? from_we : default_we();
		double default_to_re[3];
		to_we = to_we ? to_we : default_out();
		to_re = to_re ? to_re : default_to_re;

		// 正式开始计算 //
		s_inv_re2re(inv_relative_pm, from_re, to_re, from_re_type, to_re_type);

		double from_wa[3], to_wa[3];
		s_we2wa(from_re, from_we, from_wa, from_re_type);
		s_inv_wa2wa(inv_relative_pm, inv_relative_vs, from_wa, to_wa);
		s_wa2we(to_wa, to_re, to_we, to_re_type);

		return to_we;
	}
	auto s_wq2wq(const double *relative_pm, const double *relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_rq = from_rq ? from_rq : default_rq();
		from_wq = from_wq ? from_wq : default_wq();
		double default_to_rq[4];
		to_wq = to_wq ? to_wq : default_out();
		to_rq = to_rq ? to_rq : default_to_rq;

		// 正式开始计算 //
		s_rq2rq(relative_pm, from_rq, to_rq);

		double from_wa[3], to_wa[3];
		s_wq2wa(from_rq, from_wq, from_wa);
		s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);
		s_wa2wq(to_wa, to_rq, to_wq);

		return to_wq;
	}
	auto s_inv_wq2wq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_rq = from_rq ? from_rq : default_rq();
		from_wq = from_wq ? from_wq : default_wq();
		double default_to_rq[4];
		to_wq = to_wq ? to_wq : default_out();
		to_rq = to_rq ? to_rq : default_to_rq;

		// 正式开始计算 //
		s_inv_rq2rq(inv_relative_pm, from_rq, to_rq);

		double from_wa[3], to_wa[3];
		s_wq2wa(from_rq, from_wq, from_wa);
		s_inv_wa2wa(inv_relative_pm, inv_relative_vs, from_wa, to_wa);
		s_wa2wq(to_wa, to_rq, to_wq);

		return to_wq;
	}
	auto s_wm2wm(const double *relative_pm, const double *relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_rm = from_rm ? from_rm : default_rm();
		from_wm = from_wm ? from_wm : default_wm();
		double default_to_rm[9];
		to_wm = to_wm ? to_wm : default_out();
		to_rm = to_rm ? to_rm : default_to_rm;

		// 正式开始计算 //
		s_rm2rm(relative_pm, from_rm, to_rm);

		double from_wa[3], to_wa[3];
		s_wm2wa(from_rm, from_wm, from_wa);
		s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);
		s_wa2wm(to_wa, to_rm, to_wm);

		return to_wm;
	}
	auto s_inv_wm2wm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_rm = from_rm ? from_rm : default_rm();
		from_wm = from_wm ? from_wm : default_wm();
		double default_to_rm[9];
		to_wm = to_wm ? to_wm : default_out();
		to_rm = to_rm ? to_rm : default_to_rm;

		// 正式开始计算 //
		s_inv_rm2rm(inv_relative_pm, from_rm, to_rm);

		double from_wa[3], to_wa[3];
		s_wm2wa(from_rm, from_wm, from_wa);
		s_inv_wa2wa(inv_relative_pm, inv_relative_vs, from_wa, to_wa);
		s_wa2wm(to_wa, to_rm, to_wm);

		return to_wm;
	}
	auto s_wa2wa(const double *relative_pm, const double *relative_vs, const double *from_wa, double *to_wa) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_wa = from_wa ? from_wa : default_wa();
		to_wa = to_wa ? to_wa : default_out();

		// 正式开始计算 //
		s_mm(3, 1, 3, relative_pm, 4, from_wa, 1, to_wa, 1);
		s_va(3, relative_vs + 3, to_wa);

		return to_wa;
	}
	auto s_inv_wa2wa(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_wa, double *to_wa) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_pm();
		from_wa = from_wa ? from_wa : default_wa();
		to_wa = to_wa ? to_wa : default_out();

		// 正式计算开始 //
		double tem[3]{ -inv_relative_vs[3],-inv_relative_vs[4],-inv_relative_vs[5] };
		s_va(3, from_wa, tem);
		s_mm(3, 1, 3, inv_relative_pm, ColMajor{ 4 }, tem, 1, to_wa, 1);

		return to_wa;
	}
	auto s_va2va(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_va, double *to_va, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_pp = from_pp ? from_pp : default_pp();
		from_va = from_va ? from_va : default_va();
		double default_to_pp[3];
		to_pp = to_pp ? to_pp : default_to_pp;
		to_va = to_va ? to_va : default_out();

		// 正式开始计算 //
		s_pp2pp(relative_pm, from_pp, to_pp);

		double from_vs[6], to_vs[6];
		s_va2vs(from_pp, from_va, from_vs);
		s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
		s_vs2va(to_vs, to_pp, to_va);

		return to_va;
	}
	auto s_inv_va2va(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_va, double *to_va, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_pp = from_pp ? from_pp : default_pp();
		from_va = from_va ? from_va : default_va();
		double default_to_pp[3];
		to_pp = to_pp ? to_pp : default_to_pp;
		to_va = to_va ? to_va : default_out();

		// 正式开始计算 //
		s_inv_pp2pp(inv_relative_pm, from_pp, to_pp);

		double from_vs[6], to_vs[6];
		s_va2vs(from_pp, from_va, from_vs);
		s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
		s_vs2va(to_vs, to_pp, to_va);

		return to_va;
	}
	auto s_ve2ve(const double *relative_pm, const double *relative_vs, const double *from_pe, const double *from_ve, double *to_ve, double *to_pe, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_pe = from_pe ? from_pe : default_pe();
		from_ve = from_ve ? from_ve : default_ve();
		double default_to_pe[6];
		to_ve = to_ve ? to_ve : default_out();
		to_pe = to_pe ? to_pe : default_to_pe;

		// 正式开始计算 //
		s_pe2pe(relative_pm, from_pe, to_pe, from_re_type, to_re_type);

		double from_vs[6], to_vs[6];
		s_ve2vs(from_pe, from_ve, from_vs, from_re_type);
		s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
		s_vs2ve(to_vs, to_pe, to_ve, to_re_type);

		return to_ve;
	}
	auto s_inv_ve2ve(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pe, const double *from_ve, double *to_ve, double *to_pe, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_pe = from_pe ? from_pe : default_pe();
		from_ve = from_ve ? from_ve : default_ve();
		double default_to_pe[6];
		to_ve = to_ve ? to_ve : default_out();
		to_pe = to_pe ? to_pe : default_to_pe;

		// 正式开始计算 //
		s_inv_pe2pe(inv_relative_pm, from_pe, to_pe, from_re_type, to_re_type);

		double from_vs[6], to_vs[6];
		s_ve2vs(from_pe, from_ve, from_vs, from_re_type);
		s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
		s_vs2ve(to_vs, to_pe, to_ve, to_re_type);

		return to_ve;
	}
	auto s_vq2vq(const double *relative_pm, const double *relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_pq = from_pq ? from_pq : default_pq();
		from_vq = from_vq ? from_vq : default_vq();
		double default_to_pq[7];
		to_vq = to_vq ? to_vq : default_out();
		to_pq = to_pq ? to_pq : default_to_pq;

		// 正式开始计算 //
		s_pq2pq(relative_pm, from_pq, to_pq);

		double from_vs[6], to_vs[6];
		s_vq2vs(from_pq, from_vq, from_vs);
		s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
		s_vs2vq(to_vs, to_pq, to_vq);

		return to_vq;
	}
	auto s_inv_vq2vq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_pq = from_pq ? from_pq : default_pq();
		from_vq = from_vq ? from_vq : default_vq();
		double default_to_pq[7];
		to_vq = to_vq ? to_vq : default_out();
		to_pq = to_pq ? to_pq : default_to_pq;

		// 正式开始计算 //
		s_inv_pq2pq(inv_relative_pm, from_pq, to_pq);

		double from_vs[6], to_vs[6];
		s_vq2vs(from_pq, from_vq, from_vs);
		s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
		s_vs2vq(to_vs, to_pq, to_vq);

		return to_vq;
	}
	auto s_vm2vm(const double *relative_pm, const double *relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_pm = from_pm ? from_pm : default_pm();
		from_vm = from_vm ? from_vm : default_vm();
		double default_to_pm[16];
		to_vm = to_vm ? to_vm : default_out();
		to_pm = to_pm ? to_pm : default_to_pm;

		// 正式开始计算 //
		s_pm2pm(relative_pm, from_pm, to_pm);

		double from_vs[6], to_vs[6];
		s_vm2vs(from_pm, from_vm, from_vs);
		s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
		s_vs2vm(to_vs, to_pm, to_vm);

		return to_vm;
	}
	auto s_inv_vm2vm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_pm = from_pm ? from_pm : default_pm();
		from_vm = from_vm ? from_vm : default_vm();
		double default_to_pm[16];
		to_vm = to_vm ? to_vm : default_out();
		to_pm = to_pm ? to_pm : default_to_pm;

		// 正式开始计算 //
		s_inv_pm2pm(inv_relative_pm, from_pm, to_pm);

		double from_vs[6], to_vs[6];
		s_vm2vs(from_pm, from_vm, from_vs);
		s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
		s_vs2vm(to_vs, to_pm, to_vm);

		return to_vm;
	}
	auto s_vs2vs(const double *relative_pm, const double *relative_vs, const double *from_vs, double *to_vs) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		from_vs = from_vs ? from_vs : default_vs();
		to_vs = to_vs ? to_vs : default_out();

		// 正式开始计算 //
		s_tv(relative_pm, from_vs, to_vs);
		s_va(6, relative_vs, to_vs);

		return to_vs;
	}
	auto s_inv_vs2vs(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_vs, double *to_vs) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		from_vs = from_vs ? from_vs : default_vs();
		to_vs = to_vs ? to_vs : default_out();


		// 正式开始计算 //
		double tem[6];
		std::copy_n(from_vs, 6, tem);
		s_vs(6, inv_relative_vs, tem);
		s_inv_tv(inv_relative_pm, tem, to_vs);

		return to_vs;
	}

	auto s_ap2ap(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_pp = from_pp ? from_pp : default_pp();
		from_vp = from_vp ? from_vp : default_vp();
		from_ap = from_ap ? from_ap : default_ap();
		double default_to_vp[3];
		double default_to_pp[3];
		to_ap = to_ap ? to_ap : default_out();
		to_vp = to_vp ? to_vp : default_to_vp;
		to_pp = to_pp ? to_pp : default_to_pp;

		// 正式开始计算 //
		double tem_vp[3];
		s_vp2vp(relative_pm, relative_vs, from_pp, from_vp, to_vp, to_pp);

		s_c3(relative_as + 3, to_pp, to_ap);
		std::copy_n(to_vp, 3, tem_vp);
		s_mma(3, 1, 3, relative_pm, 4, from_vp, 1, tem_vp, 1);
		s_c3a(relative_vs + 3, tem_vp, to_ap);
		s_mma(3, 1, 3, relative_pm, 4, from_ap, 1, to_ap, 1);
		s_va(3, relative_as, to_ap);

		return to_ap;
	}
	auto s_inv_ap2ap(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_pp = from_pp ? from_pp : default_pp();
		from_vp = from_vp ? from_vp : default_vp();
		from_ap = from_ap ? from_ap : default_ap();
		double default_to_vp[3];
		double default_to_pp[3];
		to_ap = to_ap ? to_ap : default_out();
		to_vp = to_vp ? to_vp : default_to_vp;
		to_pp = to_pp ? to_pp : default_to_pp;

		// 正式开始计算 //
		s_inv_vp2vp(inv_relative_pm, inv_relative_vs, from_pp, from_vp, to_vp, to_pp);

		std::fill_n(to_ap, 3, 0);

		double tem[3], tem2[3];

		std::copy_n(from_ap, 3, tem);
		s_c3s(inv_relative_as + 3, from_pp, tem);

		std::copy_n(from_vp, 3, tem2);
		s_mma(3, 1, 3, inv_relative_pm, 4, to_vp, 1, tem2, 1);
		s_c3s(inv_relative_vs + 3, tem2, tem);

		s_vs(3, inv_relative_as, tem);

		s_mm(3, 1, 3, inv_relative_pm, ColMajor{ 4 }, tem, 1, to_ap, 1);

		return to_ap;
	}
	auto s_xe2xe(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_re, const double *from_we, const double *from_xe, double *to_xe, double *to_we, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_re = from_re ? from_re : default_re();
		from_we = from_we ? from_we : default_we();
		from_xe = from_xe ? from_xe : default_xe();
		double default_to_re[3];
		double default_to_we[3];
		to_re = to_re ? to_re : default_to_re;
		to_we = to_we ? to_we : default_to_we;
		to_xe = to_xe ? to_xe : default_out();

		// 正式开始计算 //
		s_re2re(relative_pm, from_re, to_re, from_re_type, to_re_type);

		double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
		s_xe2xa(from_re, from_we, from_xe, from_xa, from_wa, from_re_type);
		s_xa2xa(relative_pm, relative_vs, relative_as, from_wa, from_xa, to_xa, to_wa);
		s_xa2xe(to_wa, to_xa, to_re, to_xe, to_we, to_re_type);

		return to_xe;
	}
	auto s_inv_xe2xe(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_re, const double *from_we, const double *from_xe, double *to_xe, double *to_we, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_re = from_re ? from_re : default_re();
		from_we = from_we ? from_we : default_we();
		from_xe = from_xe ? from_xe : default_xe();
		double default_to_re[3];
		double default_to_we[3];
		to_re = to_re ? to_re : default_to_re;
		to_we = to_we ? to_we : default_to_we;
		to_xe = to_xe ? to_xe : default_out();

		// 正式开始计算 //
		s_inv_re2re(inv_relative_pm, from_re, to_re, from_re_type, to_re_type);

		double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
		s_xe2xa(from_re, from_we, from_xe, from_xa, from_wa, from_re_type);
		s_inv_xa2xa(inv_relative_pm, inv_relative_vs, inv_relative_as, from_wa, from_xa, to_xa, to_wa);
		s_xa2xe(to_wa, to_xa, to_re, to_xe, to_we, to_re_type);

		return to_xe;
	}
	auto s_xq2xq(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq, double *to_rq) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_rq = from_rq ? from_rq : default_rq();
		from_wq = from_wq ? from_wq : default_wq();
		from_xq = from_xq ? from_xq : default_xq();
		double default_to_wq[4];
		double default_to_rq[4];
		to_xq = to_xq ? to_xq : default_out();
		to_wq = to_wq ? to_wq : default_to_wq;
		to_rq = to_rq ? to_rq : default_to_rq;

		// 正式开始计算 //
		s_rq2rq(relative_pm, from_rq, to_rq);

		double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
		s_xq2xa(from_rq, from_wq, from_xq, from_xa, from_wa);
		s_xa2xa(relative_pm, relative_vs, relative_as, from_wa, from_xa, to_xa, to_wa);
		s_xa2xq(to_wa, to_xa, to_rq, to_xq, to_wq);

		return to_xq;
	}
	auto s_inv_xq2xq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq, double *to_rq) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_rq = from_rq ? from_rq : default_rq();
		from_wq = from_wq ? from_wq : default_wq();
		from_xq = from_xq ? from_xq : default_xq();
		double default_to_wq[4];
		double default_to_rq[4];
		to_xq = to_xq ? to_xq : default_out();
		to_wq = to_wq ? to_wq : default_to_wq;
		to_rq = to_rq ? to_rq : default_to_rq;

		// 正式开始计算 //
		s_inv_rq2rq(inv_relative_pm, from_rq, to_rq);

		double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
		s_xq2xa(from_rq, from_wq, from_xq, from_xa, from_wa);
		s_inv_xa2xa(inv_relative_pm, inv_relative_vs, inv_relative_as, from_wa, from_xa, to_xa, to_wa);
		s_xa2xq(to_wa, to_xa, to_rq, to_xq, to_wq);

		return to_xq;
	}
	auto s_xm2xm(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm, double *to_rm) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_rm = from_rm ? from_rm : default_rm();
		from_wm = from_wm ? from_wm : default_wm();
		from_xm = from_xm ? from_xm : default_xm();
		double default_to_rm[9];
		double default_to_wm[9];
		to_rm = to_rm ? to_rm : default_to_rm;
		to_wm = to_wm ? to_wm : default_to_wm;
		to_xm = to_xm ? to_xm : default_out();

		// 正式开始计算 //
		s_rm2rm(relative_pm, from_rm, to_rm);

		double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
		s_xm2xa(from_rm, from_wm, from_xm, from_xa, from_wa);
		s_xa2xa(relative_pm, relative_vs, relative_as, from_wa, from_xa, to_xa, to_wa);
		s_xa2xm(to_wa, to_xa, to_rm, to_xm, to_wm);

		return to_xm;
	}
	auto s_inv_xm2xm(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm, double *to_rm) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_rm = from_rm ? from_rm : default_rm();
		from_wm = from_wm ? from_wm : default_wm();
		from_xm = from_xm ? from_xm : default_xm();
		double default_to_rm[9];
		double default_to_wm[9];
		to_rm = to_rm ? to_rm : default_to_rm;
		to_wm = to_wm ? to_wm : default_to_wm;
		to_xm = to_xm ? to_xm : default_out();

		// 正式开始计算 //
		s_inv_rm2rm(inv_relative_pm, from_rm, to_rm);

		double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
		s_xm2xa(from_rm, from_wm, from_xm, from_xa, from_wa);
		s_inv_xa2xa(inv_relative_pm, inv_relative_vs, inv_relative_as, from_wa, from_xa, to_xa, to_wa);
		s_xa2xm(to_wa, to_xa, to_rm, to_xm, to_wm);

		return to_xm;
	}
	auto s_xa2xa(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_wa, const double *from_xa, double *to_xa, double *to_wa) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_wa = from_wa ? from_wa : default_wa();
		from_xa = from_xa ? from_xa : default_xa();
		double default_to_wa[3];
		to_xa = to_xa ? to_xa : default_out();
		to_wa = to_wa ? to_wa : default_to_wa;

		// 正式开始计算 //
		s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);

		s_mm(3, 1, 3, relative_pm, 4, from_xa, 1, to_xa, 1);
		s_c3a(relative_vs + 3, to_wa, to_xa);
		s_va(3, relative_as + 3, to_xa);

		return to_xa;
	}
	auto s_inv_xa2xa(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_wa, const double *from_xa, double *to_xa, double *to_wa) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_wa = from_wa ? from_wa : default_wa();
		from_xa = from_xa ? from_xa : default_xa();
		double default_to_wa[3];
		to_xa = to_xa ? to_xa : default_out();
		to_wa = to_wa ? to_wa : default_to_wa;

		// 正式开始计算 //
		s_inv_wa2wa(inv_relative_pm, inv_relative_vs, from_wa, to_wa);

		double tem[3]{ -inv_relative_as[3],-inv_relative_as[4],-inv_relative_as[5] };
		s_va(3, from_xa, tem);
		s_c3s(inv_relative_vs + 3, from_wa, tem);
		s_mm(3, 1, 3, inv_relative_pm, ColMajor{ 4 }, tem, 1, to_xa, 1);

		return to_xa;
	}
	auto s_ae2ae(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pe, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve, double *to_pe, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_pe = from_pe ? from_pe : default_pe();
		from_ve = from_ve ? from_ve : default_ve();
		from_ae = from_ae ? from_ae : default_ae();
		double default_to_ve[6];
		double default_to_pe[6];
		to_ae = to_ae ? to_ae : default_out();
		to_ve = to_ve ? to_ve : default_to_ve;
		to_pe = to_pe ? to_pe : default_to_pe;

		// 正式开始计算 //
		s_pe2pe(relative_pm, from_pe, to_pe, from_re_type, to_re_type);

		double from_vs[6], from_as[6], to_vs[6], to_as[6];
		s_ae2as(from_pe, from_ve, from_ae, from_as, from_vs, from_re_type);
		s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, to_as, to_vs);
		s_as2ae(to_vs, to_as, to_pe, to_ae, to_ve, to_re_type);

		return to_ae;
	}
	auto s_inv_ae2ae(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pe, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve, double *to_pe, const char *from_re_type, const char *to_re_type) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_pe = from_pe ? from_pe : default_pe();
		from_ve = from_ve ? from_ve : default_ve();
		from_ae = from_ae ? from_ae : default_ae();
		double default_to_ve[6];
		double default_to_pe[6];
		to_ae = to_ae ? to_ae : default_out();
		to_ve = to_ve ? to_ve : default_to_ve;
		to_pe = to_pe ? to_pe : default_to_pe;

		// 正式开始计算 //
		s_inv_pe2pe(inv_relative_pm, from_pe, to_pe, from_re_type, to_re_type);

		double from_vs[6], from_as[6], to_vs[6], to_as[6];
		s_ae2as(from_pe, from_ve, from_ae, from_as, from_vs, from_re_type);
		s_inv_as2as(inv_relative_pm, inv_relative_vs, inv_relative_as, from_vs, from_as, to_as, to_vs);
		s_as2ae(to_vs, to_as, to_pe, to_ae, to_ve, to_re_type);

		return to_ae;
	}
	auto s_aq2aq(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq, double *to_pq) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_pq = from_pq ? from_pq : default_pq();
		from_vq = from_vq ? from_vq : default_vq();
		from_aq = from_aq ? from_aq : default_aq();
		double default_to_vq[7];
		double default_to_pq[7];
		to_aq = to_aq ? to_aq : default_out();
		to_vq = to_vq ? to_vq : default_to_vq;
		to_pq = to_pq ? to_pq : default_to_pq;

		// 正式开始计算 //
		s_pq2pq(relative_pm, from_pq, to_pq);

		double from_vs[6], from_as[6], to_vs[6], to_as[6];
		s_aq2as(from_pq, from_vq, from_aq, from_as, from_vs);
		s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, to_as, to_vs);
		s_as2aq(to_vs, to_as, to_pq, to_aq, to_vq);

		return to_aq;
	}
	auto s_inv_aq2aq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq, double *to_pq) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_pq = from_pq ? from_pq : default_pq();
		from_vq = from_vq ? from_vq : default_vq();
		from_aq = from_aq ? from_aq : default_aq();
		double default_to_vq[7];
		double default_to_pq[7];
		to_aq = to_aq ? to_aq : default_out();
		to_vq = to_vq ? to_vq : default_to_vq;
		to_pq = to_pq ? to_pq : default_to_pq;

		// 正式开始计算 //
		s_inv_pq2pq(inv_relative_pm, from_pq, to_pq);

		double from_vs[6], from_as[6], to_vs[6], to_as[6];
		s_aq2as(from_pq, from_vq, from_aq, from_as, from_vs);
		s_inv_as2as(inv_relative_pm, inv_relative_vs, inv_relative_as, from_vs, from_as, to_as, to_vs);
		s_as2aq(to_vs, to_as, to_pq, to_aq, to_vq);

		return to_aq;
	}
	auto s_am2am(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm, double *to_pm) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_pm = from_pm ? from_pm : default_pm();
		from_vm = from_vm ? from_vm : default_vm();
		from_am = from_am ? from_am : default_am();
		double default_to_pm[16];
		double default_to_vm[16];
		to_am = to_am ? to_am : default_out();
		to_vm = to_vm ? to_vm : default_to_vm;
		to_pm = to_pm ? to_pm : default_to_pm;

		// 正式开始计算 //
		s_pm2pm(relative_pm, from_pm, to_pm);

		double from_vs[6], from_as[6], to_vs[6], to_as[6];
		s_am2as(from_pm, from_vm, from_am, from_as, from_vs);
		s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, to_as, to_vs);
		s_as2am(to_vs, to_as, to_pm, to_am, to_vm);

		return to_am;
	}
	auto s_inv_am2am(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm, double *to_pm) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_pm = from_pm ? from_pm : default_pm();
		from_vm = from_vm ? from_vm : default_vm();
		from_am = from_am ? from_am : default_am();
		double default_to_pm[16];
		double default_to_vm[16];
		to_am = to_am ? to_am : default_out();
		to_vm = to_vm ? to_vm : default_to_vm;
		to_pm = to_pm ? to_pm : default_to_pm;

		// 正式开始计算 //
		s_inv_pm2pm(inv_relative_pm, from_pm, to_pm);

		double from_vs[6], from_as[6], to_vs[6], to_as[6];
		s_am2as(from_pm, from_vm, from_am, from_as, from_vs);
		s_inv_as2as(inv_relative_pm, inv_relative_vs, inv_relative_as, from_vs, from_as, to_as, to_vs);
		s_as2am(to_vs, to_as, to_pm, to_am, to_vm);

		return to_am;
	}
	auto s_aa2aa(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pp, const double *from_va, const double *from_aa, double *to_aa, double *to_va, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_pp = from_pp ? from_pp : default_pp();
		from_va = from_va ? from_va : default_va();
		from_aa = from_aa ? from_aa : default_aa();
		double default_to_pp[3];
		double default_to_va[6];
		to_aa = to_aa ? to_aa : default_out();
		to_va = to_va ? to_va : default_to_va;
		to_pp = to_pp ? to_pp : default_to_pp;

		// 正式开始计算 //
		s_pp2pp(relative_pm, from_pp, to_pp);

		double from_vs[6], from_as[6], to_vs[6], to_as[6];
		s_aa2as(from_pp, from_va, from_aa, from_as, from_vs);
		s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, to_as, to_vs);
		s_as2aa(to_vs, to_as, to_pp, to_aa, to_va);

		return to_aa;
	}
	auto s_inv_aa2aa(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pp, const double *from_va, const double *from_aa, double *to_aa, double *to_va, double *to_pp) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_pp = from_pp ? from_pp : default_pp();
		from_va = from_va ? from_va : default_va();
		from_aa = from_aa ? from_aa : default_aa();
		double default_to_pp[3];
		double default_to_ve[6];
		to_aa = to_aa ? to_aa : default_out();
		to_va = to_va ? to_va : default_to_ve;
		to_pp = to_pp ? to_pp : default_to_pp;

		// 正式开始计算 //
		s_inv_pp2pp(inv_relative_pm, from_pp, to_pp);

		double from_vs[6], from_as[6], to_vs[6], to_as[6];
		s_aa2as(from_pp, from_va, from_aa, from_as, from_vs);
		s_inv_as2as(inv_relative_pm, inv_relative_vs, inv_relative_as, from_vs, from_as, to_as, to_vs);
		s_as2aa(to_vs, to_as, to_pp, to_aa, to_va);

		return to_aa;
	}
	auto s_as2as(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_vs, const double *from_as, double *to_as, double *to_vs) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		relative_vs = relative_vs ? relative_vs : default_vs();
		relative_as = relative_as ? relative_as : default_as();
		from_vs = from_vs ? from_vs : default_vs();
		from_as = from_as ? from_as : default_as();
		double default_to_vs[6];
		to_vs = to_vs ? to_vs : default_to_vs;
		to_as = to_as ? to_as : default_out();

		// 正式开始计算 //
		s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
		s_cv(relative_vs, to_vs, to_as);
		s_tva(relative_pm, from_as, to_as);
		s_va(6, relative_as, to_as);

		return to_as;
	}
	auto s_inv_as2as(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_vs, const double *from_as, double *to_as, double *to_vs) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
		inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
		from_vs = from_vs ? from_vs : default_vs();
		from_as = from_as ? from_as : default_as();
		double default_to_vs[6];
		to_vs = to_vs ? to_vs : default_to_vs;
		to_as = to_as ? to_as : default_out();

		// 正式开始计算 //
		s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
		double tem[6];
		std::fill_n(to_as, 6, 0);
		std::copy_n(from_as, 6, tem);
		s_vs(6, inv_relative_as, tem);
		s_cvs(inv_relative_vs, from_vs, tem);
		s_inv_tv(inv_relative_pm, tem, to_as);

		return to_as;
	}

	auto s_fs2fs(const double *relative_pm, const double *from_fs, double *to_fs) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_fs = from_fs ? from_fs : default_fs();
		to_fs = to_fs ? to_fs : default_out();

		// 正式开始计算 //
		s_tf(relative_pm, from_fs, to_fs);

		return to_fs;
	}
	auto s_inv_fs2fs(const double *inv_relative_pm, const double *from_fs, double *to_fs) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_fs = from_fs ? from_fs : default_fs();
		to_fs = to_fs ? to_fs : default_out();

		// 正式开始计算 //
		s_inv_tf(inv_relative_pm, from_fs, to_fs);

		return to_fs;
	}
	auto s_im2im(const double *relative_pm, const double *from_im, double *to_im) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_im = from_im ? from_im : default_im();
		to_im = to_im ? to_im : default_out();

		// It = Tf * If * (Tf)^T
		//
		// I = [ m   -cx ]
		//     [ cx   I  ]
		//   = [  m               cz -cy ]
		//     |      m      -cz      cx |
		//     |          m   cy -cx     |
		//     |     -cz  cy Ixx Ixy Ixz |
		//     |  cz     -cx Ixy Iyy Iyz |
		//     [ -cy  cx     Ixz Iyz Izz ]
		//
		// Tf = [ R     ]
		//      [ pxR R ]
		//
		// It = Tf * If * Tf^T
		//    = [ m              |  - m*px - (R*c)x                                           ]
		//      [ m*px + (R*c)x  |  - m*px*px - (R*c)x*px - px*(R*c)x + R*I*R^T               ]
		//    = [ m              |  - m*px - (R*c)x                                           ]
		//      [ m*px + (R*c)x  |  - m*px*px - (R*c)x*px - px*(R*c)x + R*I*R^T               ]
		//    = [ m              |  - (R*c + m*p)x                                            ]
		//      [ (R*c + m*p)x   |  - m*px*px - p*(R*c)^T - (R*c)*p^T + 2*(R*c)^T*p + R*I*R^T ]  

		std::fill(to_im, to_im + 36, 0.0);

		const double &m = from_im[0];
		const double &x = relative_pm[3];
		const double &y = relative_pm[7];
		const double &z = relative_pm[11];

		// R*c 
		double rc[3];
		rc[0] = relative_pm[0] * from_im[11] + relative_pm[1] * from_im[15] + relative_pm[2] * from_im[4];
		rc[1] = relative_pm[4] * from_im[11] + relative_pm[5] * from_im[15] + relative_pm[6] * from_im[4];
		rc[2] = relative_pm[8] * from_im[11] + relative_pm[9] * from_im[15] + relative_pm[10] * from_im[4];

		// R*c + m*p
		const double cx = rc[0] + m * relative_pm[3];
		const double cy = rc[1] + m * relative_pm[7];
		const double cz = rc[2] + m * relative_pm[11];

		// left top corner //
		to_im[0] = m;
		to_im[7] = m;
		to_im[14] = m;

		// right top corner //
		to_im[4] = cz;
		to_im[5] = -cy;
		to_im[9] = -cz;
		to_im[11] = cx;
		to_im[15] = cy;
		to_im[16] = -cx;

		// left bottom corner //
		to_im[19] = -cz;
		to_im[20] = cy;
		to_im[24] = cz;
		to_im[26] = -cx;
		to_im[30] = -cy;
		to_im[31] = cx;

		// right bottom corner //

		// R = [ r11 r12 r13 ]
		//     | r21 r22 r23 |
		//     [ r31 r32 r33 ]
		// I = [ Ixx Ixy Ixz ]
		//     | Ixy Iyy Iyz |
		//     [ Ixz Iyz Izz ]
		//
		// R * I = [ r11 r12 r13 ]     [ Ixx Ixy Ixz ]     [ r11 r21 r31 ]
		//         | r21 r22 r23 |  *  | Ixy Iyy Iyz |  *  | r12 r22 r32 |
		//         [ r31 r32 r33 ]     [ Ixz Iyz Izz ]     [ r13 r23 r33 ]
		//       = [ r11*Ixx + r12*Ixy + r13*Ixz     r11*Ixy + r12*Iyy + r13*Iyz     r11*Ixz + r12*Iyz + r13*Izz ]     [ r11 r21 r31 ]
		//         [ r21*Ixx + r22*Ixy + r23*Ixz     r21*Ixy + r22*Iyy + r23*Iyz     r21*Ixz + r22*Iyz + r23*Izz ]  *  | r12 r22 r32 |
		//         [ r31*Ixx + r32*Ixy + r33*Ixz     r31*Ixy + r32*Iyy + r33*Iyz     r31*Ixz + r32*Iyz + r33*Izz ]     [ r13 r23 r33 ]
		// tbd

		// R*I*R^T
		double RI[9]{
			relative_pm[0] * from_im[21] + relative_pm[1] * from_im[27] + relative_pm[2] * from_im[33],
			relative_pm[0] * from_im[22] + relative_pm[1] * from_im[28] + relative_pm[2] * from_im[34],
			relative_pm[0] * from_im[23] + relative_pm[1] * from_im[29] + relative_pm[2] * from_im[35],
			relative_pm[4] * from_im[21] + relative_pm[5] * from_im[27] + relative_pm[6] * from_im[33],
			relative_pm[4] * from_im[22] + relative_pm[5] * from_im[28] + relative_pm[6] * from_im[34],
			relative_pm[4] * from_im[23] + relative_pm[5] * from_im[29] + relative_pm[6] * from_im[35],
			relative_pm[8] * from_im[21] + relative_pm[9] * from_im[27] + relative_pm[10] * from_im[33],
			relative_pm[8] * from_im[22] + relative_pm[9] * from_im[28] + relative_pm[10] * from_im[34],
			relative_pm[8] * from_im[23] + relative_pm[9] * from_im[29] + relative_pm[10] * from_im[35],
		};

		to_im[21] = RI[0] * relative_pm[0] + RI[1] * relative_pm[1] + RI[2] * relative_pm[2];
		to_im[28] = RI[3] * relative_pm[4] + RI[4] * relative_pm[5] + RI[5] * relative_pm[6];
		to_im[35] = RI[6] * relative_pm[8] + RI[7] * relative_pm[9] + RI[8] * relative_pm[10];

		to_im[27] = RI[3] * relative_pm[0] + RI[4] * relative_pm[1] + RI[5] * relative_pm[2];
		to_im[33] = RI[6] * relative_pm[0] + RI[7] * relative_pm[1] + RI[8] * relative_pm[2];
		to_im[34] = RI[6] * relative_pm[4] + RI[7] * relative_pm[5] + RI[8] * relative_pm[6];

		// R*I*R^T - m*px*px - (R*c)*p^T - p*(R*c)^T + 2*(R*c)^T*p
		to_im[21] += m * (y*y + z * z) + 2.0*(rc[1] * y + rc[2] * z);
		to_im[28] += m * (x*x + z * z) + 2.0*(rc[0] * x + rc[2] * z);
		to_im[35] += m * (y*y + x * x) + 2.0*(rc[0] * x + rc[1] * y);

		to_im[27] -= m * (x*y) + rc[0] * y + rc[1] * x;
		to_im[33] -= m * (x*z) + rc[0] * z + rc[2] * x;
		to_im[34] -= m * (y*z) + rc[1] * z + rc[2] * y;

		// make symetric part
		to_im[22] = to_im[27];
		to_im[23] = to_im[33];
		to_im[29] = to_im[34];

		return to_im;
	}
	auto s_inv_im2im(const double *inv_relative_pm, const double *from_im, double *to_im) noexcept->double *
	{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_im = from_im ? from_im : default_im();
		to_im = to_im ? to_im : default_out();

		double pm[16];
		s_inv_pm(inv_relative_pm, pm);

		s_im2im(pm, from_im, to_im);

		return to_im;
	}
	auto s_iv2iv(const double *relative_pm, const double *from_iv, double *to_iv) noexcept->double *
	{
		// 补充默认参数 //
		relative_pm = relative_pm ? relative_pm : default_pm();
		from_iv = from_iv ? from_iv : default_iv();
		to_iv = to_iv ? to_iv : default_out();

		const double &x = relative_pm[3];
		const double &y = relative_pm[7];
		const double &z = relative_pm[11];

		// m //
		to_iv[0] = from_iv[0];

		// R*c 
		to_iv[1] = relative_pm[0] * from_iv[1] + relative_pm[1] * from_iv[2] + relative_pm[2] * from_iv[3];
		to_iv[2] = relative_pm[4] * from_iv[1] + relative_pm[5] * from_iv[2] + relative_pm[6] * from_iv[3];
		to_iv[3] = relative_pm[8] * from_iv[1] + relative_pm[9] * from_iv[2] + relative_pm[10] * from_iv[3];

		// make last 6 inertia //

		// R = [ r11 r12 r13 ]
		//     | r21 r22 r23 |
		//     [ r31 r32 r33 ]
		// I = [ Ixx Ixy Ixz ]
		//     | Ixy Iyy Iyz |
		//     [ Ixz Iyz Izz ]
		//
		// R * I * R = [ r11 r12 r13 ]     [ Ixx Ixy Ixz ]     [ r11 r21 r31 ]
		//             | r21 r22 r23 |  *  | Ixy Iyy Iyz |  *  | r12 r22 r32 |
		//             [ r31 r32 r33 ]     [ Ixz Iyz Izz ]     [ r13 r23 r33 ]
		//           = [ r11*Ixx + r12*Ixy + r13*Ixz     r11*Ixy + r12*Iyy + r13*Iyz     r11*Ixz + r12*Iyz + r13*Izz ]     [ r11 r21 r31 ]
		//             | r21*Ixx + r22*Ixy + r23*Ixz     r21*Ixy + r22*Iyy + r23*Iyz     r21*Ixz + r22*Iyz + r23*Izz |  *  | r12 r22 r32 |
		//             [ r31*Ixx + r32*Ixy + r33*Ixz     r31*Ixy + r32*Iyy + r33*Iyz     r31*Ixz + r32*Iyz + r33*Izz ]     [ r13 r23 r33 ]
		// tbd

		// R*I*R^T
		double RI[9]{
			relative_pm[0] * from_iv[4] + relative_pm[1] * from_iv[7] + relative_pm[2] * from_iv[8],
			relative_pm[0] * from_iv[7] + relative_pm[1] * from_iv[5] + relative_pm[2] * from_iv[9],
			relative_pm[0] * from_iv[8] + relative_pm[1] * from_iv[9] + relative_pm[2] * from_iv[6],
			relative_pm[4] * from_iv[4] + relative_pm[5] * from_iv[7] + relative_pm[6] * from_iv[8],
			relative_pm[4] * from_iv[7] + relative_pm[5] * from_iv[5] + relative_pm[6] * from_iv[9],
			relative_pm[4] * from_iv[8] + relative_pm[5] * from_iv[9] + relative_pm[6] * from_iv[6],
			relative_pm[8] * from_iv[4] + relative_pm[9] * from_iv[7] + relative_pm[10] * from_iv[8],
			relative_pm[8] * from_iv[7] + relative_pm[9] * from_iv[5] + relative_pm[10] * from_iv[9],
			relative_pm[8] * from_iv[8] + relative_pm[9] * from_iv[9] + relative_pm[10] * from_iv[6],
		};

		to_iv[4] = RI[0] * relative_pm[0] + RI[1] * relative_pm[1] + RI[2] * relative_pm[2];
		to_iv[5] = RI[3] * relative_pm[4] + RI[4] * relative_pm[5] + RI[5] * relative_pm[6];
		to_iv[6] = RI[6] * relative_pm[8] + RI[7] * relative_pm[9] + RI[8] * relative_pm[10];

		to_iv[7] = RI[3] * relative_pm[0] + RI[4] * relative_pm[1] + RI[5] * relative_pm[2];
		to_iv[8] = RI[6] * relative_pm[0] + RI[7] * relative_pm[1] + RI[8] * relative_pm[2];
		to_iv[9] = RI[6] * relative_pm[4] + RI[7] * relative_pm[5] + RI[8] * relative_pm[6];

		// R*I*R^T - m*px*px - (R*c)*p^T - p*(R*c)^T + 2*(R*c)^T*p
		to_iv[4] += to_iv[0] * (y*y + z * z) + 2.0*(to_iv[2] * y + to_iv[3] * z);
		to_iv[5] += to_iv[0] * (x*x + z * z) + 2.0*(to_iv[1] * x + to_iv[3] * z);
		to_iv[6] += to_iv[0] * (y*y + x * x) + 2.0*(to_iv[1] * x + to_iv[2] * y);

		to_iv[7] -= to_iv[0] * (x*y) + to_iv[1] * y + to_iv[2] * x;
		to_iv[8] -= to_iv[0] * (x*z) + to_iv[1] * z + to_iv[3] * x;
		to_iv[9] -= to_iv[0] * (y*z) + to_iv[2] * z + to_iv[3] * y;

		// R*c + m*p
		to_iv[1] += to_iv[0] * relative_pm[3];
		to_iv[2] += to_iv[0] * relative_pm[7];
		to_iv[3] += to_iv[0] * relative_pm[11];

		return to_iv;
	}
	auto s_inv_iv2iv(const double *inv_relative_pm, const double *from_iv, double *to_iv) noexcept->double *{
		// 补充默认参数 //
		inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
		from_iv = from_iv ? from_iv : default_iv();
		to_iv = to_iv ? to_iv : default_out();

		double pm[16];
		s_inv_pm(inv_relative_pm, pm);
		s_iv2iv(pm, from_iv, to_iv);

		return to_iv;
	}

	auto s_sov_pnts2pm(const double *origin, Size origin_ld, const double *first_pnt, Size first_ld, const double *second_pnt, Size second_ld, double *pm_out, const char *axis_order) noexcept->void{
		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		// 以下求解位置 //
		s_vc(3, origin, origin_ld, pm_out + 3, 4);

		// 以下求解角度 //
		int order[3]{ axis_order[0] - 'x', axis_order[1] - 'x', 3 + 'x' + 'x' - axis_order[0] - axis_order[1] };

		s_vc(3, first_pnt, first_ld, pm_out + order[0], 4);
		s_vc(3, second_pnt, second_ld, pm_out + order[1], 4);

		s_vs(3, origin, origin_ld, pm_out + order[0], 4);
		s_vs(3, origin, origin_ld, pm_out + order[1], 4);

		double alpha = (((order[1] - order[0] + 3) % 3) == 1) ? 1.0 : -1.0;

		s_c3(alpha, pm_out + order[0], 4, pm_out + order[1], 4, pm_out + order[2], 4);

		double nrm = s_norm(3, pm_out + order[2], 4);
		if (nrm == 0)
		{
			if (s_norm(3, pm_out + order[0], 4) == 0)
			{
				s_mc(3, 3, default_rm(), 3, pm_out, 4);
				return;
			}
			else
			{
				s_nv(3, 1.0 / s_norm(3, pm_out + order[0], 4), pm_out + order[0], 4);

				double rm[9];
				s_c3_n(3, pm_out + order[0], 4, default_rm(), 3, rm, 3);
				double norm[3];
				norm[0] = s_norm(3, rm, 3);
				norm[1] = s_norm(3, rm + 1, 3);
				norm[2] = s_norm(3, rm + 2, 3);

				Size max_id = std::max_element(norm, norm + 3) - norm;
				s_vc(3, 1.0 / norm[max_id], rm + max_id, 3, pm_out + order[1], 4);
				s_c3(alpha, pm_out + order[0], 4, pm_out + order[1], 4, pm_out + order[2], 4);
			}
		}
		else
		{
			s_nv(3, 1.0 / nrm, pm_out + order[2], 4);
			s_nv(3, 1.0 / s_norm(3, pm_out + order[0], 4), pm_out + order[0], 4);

			s_c3(alpha, pm_out + order[2], 4, pm_out + order[0], 4, pm_out + order[1], 4);
		}
	}
	auto s_sov_axes2pm(const double *origin, Size origin_ld, const double *first_axis, Size first_ld, const double *second_axis, Size second_ld, double *pm_out, const char *axis_order) noexcept->void{
		double origin_zero[3]{ 0,0,0 };

		s_sov_pnts2pm(origin_zero, 1, first_axis, first_ld, second_axis, second_ld, pm_out, axis_order);
		s_vc(3, origin, origin_ld, pm_out + 3, 4);
	}
	auto s_sov_theta(double k1, double k2, double b, double *theta_out)noexcept->int{
		double K = std::sqrt(k1 * k1 + k2 * k2);
		double rhs = b / K;

		if (std::abs(rhs) > 1.0) {
			return -1;
		} 
		else if (std::abs(rhs) < 0.7) {
			double alpha_plus_theta = std::asin(rhs);
			double alpha = std::atan2(k2, k1);
			theta_out[0] = alpha_plus_theta - alpha;
			theta_out[1] = PI - alpha_plus_theta - alpha;
		} 
		else {
			double alpha_plus_theta = std::acos(rhs);
			double alpha = std::atan2(-k1, k2);
			theta_out[0] = alpha_plus_theta - alpha;
			theta_out[1] = -alpha_plus_theta - alpha;
		}

		if (theta_out[0] > PI)theta_out[0] -= 2 * PI;
		if (theta_out[1] > PI)theta_out[1] -= 2 * PI;
		if (theta_out[0] < -PI)theta_out[0] += 2 * PI;
		if (theta_out[1] < -PI)theta_out[1] += 2 * PI;

		return 0;
	}
	auto s_sov_ab(const double*pp, double *ab, const char*order)noexcept->void{
		// 补充默认参数 //
		static const double default_pp[3]{ 1,0,0 };
		double default_ab[3];
		pp = pp ? pp : default_pp;
		ab = ab ? ab : default_ab;

		// 正式开始计算 //
		const Size a = order[0] - '1';
		const Size b = order[1] - '1';
		const Size c = 3 - a - b;
		const double pa = pp[a];
		const double pb = pp[b];
		const double pc = pp[c];
		const double Pbc = P()[b][c];
		const double Pac = P()[a][c];

		const double k = std::sqrt(pb * pb + pc * pc);

		ab[0] = std::atan2(Pbc * pb, pc);
		ab[1] = std::atan2(Pac * pa, k);
	}
	auto s_sov_vab(const double*pp, const double*vp, double *vab, double *ab, const char*order)noexcept->void{
		// 补充默认参数 //
		static const double default_pp[3]{ 1,0,0 };
		static const double default_vp[3]{ 0,0,0 };
		double default_vab[3], default_ab[3];
		pp = pp ? pp : default_pp;
		vp = vp ? vp : default_vp;
		vab = vab ? vab : default_vab;
		ab = ab ? ab : default_ab;

		// 正式开始计算 //
		const Size a = Size(order[0] - '1');
		const Size b = Size(order[1] - '1');
		const Size c = 3 - a - b;
		const double pa = pp[a];
		const double pb = pp[b];
		const double pc = pp[c];
		const double Pbc = P()[b][c];
		const double Pac = P()[a][c];

		const double k = std::sqrt(pb * pb + pc * pc);
		ab[0] = std::atan2(Pbc * pb, pc);
		ab[1] = std::atan2(Pac * pa, k);

		const double c1 = std::cos(ab[0]);
		const double c2 = std::cos(ab[1]);
		const double vpa = vp[a];
		const double vpb = vp[b];
		const double vpc = vp[c];

		const double vk = (pb*vpb + pc * vpc) / k;
		const double q1 = vpb * pc - vpc * pb;
		const double q2 = vpa * k - vk * pa;
		vab[0] = Pbc * q1*c1*c1 / (pc*pc);
		vab[1] = Pac * q2*c2*c2 / (k*k);
	}
	auto s_sov_aab(const double*pp, const double*vp, const double*ap, double *aab, double *vab, double *ab, const char*order)noexcept->void{
		// 补充默认参数 //
		static const double default_pp[3]{ 1,0,0 };
		static const double default_vp[3]{ 0,0,0 };
		static const double default_ap[3]{ 0,0,0 };
		double default_aab[3], default_vab[3], default_ab[3];
		pp = pp ? pp : default_pp;
		vp = vp ? vp : default_vp;
		ap = ap ? ap : default_ap;
		aab = vab ? aab : default_aab;
		vab = vab ? vab : default_vab;
		ab = ab ? ab : default_ab;

		// 正式开始计算 //
		const Size a = Size(order[0] - '1');
		const Size b = Size(order[1] - '1');
		const Size c = 3 - a - b;
		const double pa = pp[a];
		const double pb = pp[b];
		const double pc = pp[c];
		const double Pbc = P()[b][c];
		const double Pac = P()[a][c];

		const double k = std::sqrt(pb * pb + pc * pc);
		ab[0] = std::atan2(Pbc * pb, pc);
		ab[1] = std::atan2(Pac * pa, k);

		const double c1 = std::cos(ab[0]);
		const double c2 = std::cos(ab[1]);
		const double vpa = vp[a];
		const double vpb = vp[b];
		const double vpc = vp[c];

		const double vk = (pb*vpb + pc * vpc) / k;
		const double q1 = vpb * pc - vpc * pb;
		const double q2 = vpa * k - vk * pa;
		vab[0] = Pbc * q1*c1*c1 / (pc*pc);
		vab[1] = Pac * q2*c2*c2 / (k*k);

		const double s1 = std::sin(ab[0]);
		const double s2 = std::sin(ab[1]);
		const double apa = ap[a];
		const double apb = ap[b];
		const double apc = ap[c];
		const double ak = (pb*apb + vpb * vpb + pc * apc + vpc * vpc - vk * vk) / k;
		const double vq1 = apb * pc - apc * pb;
		const double vq2 = apa * k - ak * pa;

		aab[0] = Pbc * ((vq1*c1*c1 - 2 * q1*c1*s1*vab[0])*pc - 2 * vpc*q1*c1*c1) / (pc*pc*pc);
		aab[1] = Pac * ((vq2*c2*c2 - 2 * q2*c2*s2*vab[1])*k - 2 * vk*q2*c2*c2) / (k*k*k);
	}
	auto s_sov_ab_arbitrary(const double*pp0, const double *pp, double *alpha, double *beta, const char*order)noexcept->int {
		// 补充默认参数 //
		static const double default_pp[3]{ 1,0,0 };
		pp0 = pp0 ? pp0 : default_pp;
		pp = pp ? pp : default_pp;

		// 正式开始计算 //
		const Size a = Size(order[0] - '1');
		const Size b = Size(order[1] - '1');
		const Size c = 3 - a - b;
		const double xa = pp0[a];
		const double xb = pp0[b];
		const double xc = pp0[c];
		const double ya = pp[a];
		const double yb = pp[b];
		const double yc = pp[c];
		const double Pbc = P()[b][c];
		const double Pcb = P()[c][b];
		const double Pac = P()[a][c];
		const double Pca = P()[c][a];

		if(s_sov_theta(Pac*xc, xa, ya, beta))return -1;
		for (int i = 0; i < 2; ++i)
		{
			const auto s2 = std::sin(beta[i]);
			const auto c2 = std::cos(beta[i]);

			const auto k1 = Pbc * Pca * s2 * xa + Pbc * c2 * xc;
			const auto k2 = xb;
			const auto k3 = Pcb * xb;
			const auto k4 = Pca * s2 * xa + c2 * xc;



			// 符号会影响 atan2 的计算 //
			const auto sig = s_sgn2(k1 * k4 - k2 * k3);
			alpha[i] = std::atan2((k4 * yb - k2 * yc)*sig, (k1 * yc - k3 * yb)*sig);
		}




		return 0;
	}
	auto s_sov_axis_distance(const double*from_pm, const double*to_pm, Size axis)noexcept->double{
		if (axis < 3){
			double dx{ to_pm[3] - from_pm[3] }, dy{ to_pm[7] - from_pm[7] }, dz{ to_pm[11] - from_pm[11] };
			return from_pm[axis] * dx + from_pm[axis + 4] * dy + from_pm[axis + 8] * dz;
		}else{
			Size b{ (axis - 2) % 3 }, c{ (axis - 1) % 3 };

			double Pbb = from_pm[b] * to_pm[b] + from_pm[b + 4] * to_pm[b + 4] + from_pm[b + 8] * to_pm[b + 8];
			double Pcc = from_pm[c] * to_pm[c] + from_pm[c + 4] * to_pm[c + 4] + from_pm[c + 8] * to_pm[c + 8];
			double Pbc = from_pm[b] * to_pm[c] + from_pm[b + 4] * to_pm[c + 4] + from_pm[b + 8] * to_pm[c + 8];
			double Pcb = from_pm[c] * to_pm[b] + from_pm[c + 4] * to_pm[b + 4] + from_pm[c + 8] * to_pm[b + 8];

			return std::atan2(Pcb - Pbc, Pbb + Pcc);
		}
	}

	auto s_calib_tool_two_pnts(const double* input, double*result, double mini_angle)noexcept->int {
		// check diff angle
		auto diff = input[5] - input[2];

		while (diff > aris::PI) diff -= 2 * aris::PI;
		while (diff < -aris::PI)diff += 2 * aris::PI;

		if (std::abs(diff) < mini_angle)return -1;

		auto c1 = std::cos(input[2]);
		auto s1 = std::sin(input[2]);
		auto c2 = std::cos(input[5]);
		auto s2 = std::sin(input[5]);

		const double A[4]{c1 - c2, -s1 + s2, s1 - s2, c1 - c2 };
		const double tem = 1.0 / (A[0] * A[3] - A[1] * A[2]);
		
		double inv_A[4]{A[3]*tem, -A[1]*tem, -A[2]*tem, A[0]*tem};

		double b[2]{ input[3] - input[0], input[4] - input[1] };
		s_mm(2, 1, 2, inv_A, b, result);
		return 0;
 	}
}
