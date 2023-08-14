#include"aris/plan/scurve.hpp"

//#define DEBUG_ARIS_PLAN_TRAJECTORY


namespace aris::plan {
    auto inline safe_sqrt(double v)->double {
        return v > 0.0 ? std::sqrt(v) : 0.0;
    }
    auto newton_raphson_binary_search(std::function<double(double)> f, double x_below, double x_upper) -> double {
        double f_upper = f(x_upper);
        double f_below = f(x_below);

        double fsig = aris::dynamic::s_sgn2(f_upper - f_below);
        double xsig = aris::dynamic::s_sgn2(x_upper - x_below);

        if (aris::dynamic::s_sgn2(f_upper * f_below) >= 0)
            return std::abs(f_upper) < std::abs(f_below) ? x_upper : x_below;

        double diff = std::abs(x_upper - x_below);
        double diff_last = 10 * diff;

        while (diff < diff_last) {
            diff_last = diff;

            double x_mid = x_below + (x_upper - x_below) / 2;
            double f_mid = f(x_mid);

            if (aris::dynamic::s_sgn2(f_mid) == fsig) {
                x_upper = x_mid;
                f_upper = f_mid;
            }
            else {
                x_below = x_mid;
                f_below = f_mid;
            }

            double x1 = (x_mid * f_below - x_below * f_mid) / (f_below - f_mid);
            if (xsig * x1 <= xsig * x_upper && xsig * x1 >= xsig * x_below) {
                double fx1 = f(x1);
                if (aris::dynamic::s_sgn2(fx1) == fsig) {
                    x_upper = x1;
                    f_upper = fx1;
                }
                else {
                    x_below = x1;
                    f_below = fx1;
                }
            }

            double x2 = (x_mid * f_upper - x_upper * f_mid) / (f_upper - f_mid);
            if (xsig * x2 <= xsig * x_upper && xsig * x2 >= xsig * x_below) {
                double fx2 = f(x2);
                if (aris::dynamic::s_sgn2(fx2) == fsig) {
                    x_upper = x2;
                    f_upper = fx2;
                }
                else {
                    x_below = x2;
                    f_below = fx2;
                }

            }

            diff = std::abs(x_upper - x_below);
        }
        return (x_below + x_upper) / 2;
    }

    struct TRange {
        double below_, upper_;
    };
    using TSet = std::vector<TRange>;
    // 对两个集合求交集 //
    auto s_scurve_cpt_intersection(TSet& set1, TSet& set2, double error_allow = 0) -> TSet {
        TSet ret;
        for (auto& r1 : set1) {
            for (auto& r2 : set2) {
                if (r2.upper_ < r1.below_ - error_allow || r2.below_ > r1.upper_ + error_allow)
                    continue;
                else {
                    double below = std::max(r1.below_, r2.below_);
                    double upper = std::min(r1.upper_, r2.upper_);
                    ret.push_back(TRange{std::min(below, upper), std::max(below, upper)});
                };
            }
        }
        return ret;
    }

    // 给定时间，计算加加速段的终止速度
    auto s_acc_vend(double va, double a, double j, double T)noexcept->double {
        return a / j > T / 2.0 ? va + j * T * T / 4.0 : va + T * a - a * a / j;
    };
    // 给定速度，计算加加速段的终止速度
    auto s_acc_time(double va, double vb, double a, double j)noexcept->double {
        double v_diff = std::abs(vb - va);
        return v_diff > a * a / j ? v_diff / a + a / j : 2 * safe_sqrt(v_diff / j);
    };

    // 不考虑 pt //
    auto s_scurve_cpt_vc_upper_by_va_vb_T(double va, double vb, double T, double a, double j, double *vc_upper = nullptr, double *Ta = nullptr, double *Tb = nullptr) -> double {
        double cons = 1e-10;

        double vc_upper_value_{ 0.0 }, Ta_value_{ 0.0 }, Tb_value_{ 0.0 };
        vc_upper = vc_upper ? vc_upper : &vc_upper_value_;
        Ta = Ta ? Ta : &Ta_value_;
        Tb = Tb ? Tb : &Tb_value_;

        double v1 = std::min(va, vb);
        double v2 = std::max(va, vb);
        double ve1 = v1 + a * a / j;
        double ve2 = v2 + a * a / j;

        double T_v1_to_v2 = s_acc_time(v1, v2, a, j);
        double T_v1_to_ve1 = s_acc_time(v1, ve1, a, j);
        double T_v2_to_ve1 = s_acc_time(v2, ve1, a, j);
        double T_v1_to_ve2 = s_acc_time(v1, ve2, a, j);
        double T_v2_to_ve2 = s_acc_time(v2, ve2, a, j);

        double vc, T1, T2;

        //% v1 无法加速到 v2
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        if (v2 - v1 > cons && T_v1_to_v2 > T + cons) {
            THROW_FILE_LINE("failed in s_scurve_cpt_vc_upper_by_va_vb_T");
        }
#endif
        if (T <= T_v1_to_v2) {
            vc = v2;
            T1 = T;
            T2 = 0;
        }
        else if (ve1 >= v2 && T <= T_v1_to_ve1 + T_v2_to_ve1) {
            //% 第一段无匀加速，第二段无匀加速
            //% clear
            //% syms T1 T2 T v1 v2 vc a j pt
            //% T1 = 2 * sqrt((vc - v1) / j)
            //% T2 = 2 * sqrt((vc - v2) / j)
            //% vc_ans = solve(T1 + T2 == T, vc)
            vc = (T*T*j + 8*v1 + 8*v2)/16 + (v2-v1)/T * (v2-v1)/T / j;
            T1 = 2 * safe_sqrt((vc - v1) / j);
            T2 = T - T1;
        }
        else if (T <= T_v1_to_ve2 + T_v2_to_ve2) {
            //% 第一段有匀加速，第二段无匀加速
            //% clear
            //% syms T1 T2 T v1 v2 vc a j pt
            //% T1 = (vc - v1) / a + a / j
            //% T2 = 2 * sqrt((vc - v2) / j)
            //% solve(T1 + T2 == T, vc)
            vc = (j * v1 + a*a - 2 * a * j * safe_sqrt((v1 - v2 + T * a) / j) + T * a * j) / j;
            T1 = (vc - v1) / a + a / j;
            T2 = T - T1;
        }
        else {
            //% 第一段有匀加速，第二段有匀加速
            //% clear
            //% syms T1 T2 T v1 v2 vc a j pt
            //% T1 = (vc - v1) / a + a / j
            //% T2 = (vc - v2) / a + a / j
            //% solve(T1 + T2 == T, vc)
            vc = (-2 * a*a + T * j * a + j * v1 + j * v2) / (2 * j);
            T1 = (vc - v1) / a + a / j;
            T2 = T - T1;
        }

        //% 更新 vc_upper, Ta, Tb
        *vc_upper = vc;
        if (vb > va) {
            *Ta = T1;
            *Tb = T2;
        }
        else {
            *Ta = T2;
            *Tb = T1;
        }

        return *vc_upper;
    }
    auto s_scurve_cpt_vc_below_by_va_vb_T(double va, double vb, double T, double a, double j, double* vc_below = nullptr, double* Ta = nullptr, double* Tb = nullptr)->double {
        double cons = 1e-10;

        double vc_below_value_, Ta_value_, Tb_value_;
        vc_below = vc_below ? vc_below : &vc_below_value_;
        Ta = Ta ? Ta : &Ta_value_;
        Tb = Tb ? Tb : &Tb_value_;

        double v1 = std::min(va, vb);
        double v2 = std::max(va, vb);
        double ve1 = v1 - a * a / j;
        double ve2 = v2 - a * a / j;

        double T_v1_to_v2 = s_acc_time(v1, v2, a, j);
        double T_v1_to_ve1 = s_acc_time(v1, ve1, a, j);
        double T_v2_to_ve1 = s_acc_time(v2, ve1, a, j);
        double T_v1_to_ve2 = s_acc_time(v1, ve2, a, j);
        double T_v2_to_ve2 = s_acc_time(v2, ve2, a, j);

        double vc, T1, T2;

        //% v1 无法加速到 v2
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        if (v2 - v1 > cons && T_v1_to_v2 > T + cons) {
            THROW_FILE_LINE("failed in s_scurve_cpt_vc_below_by_va_vb_T");
        }
#endif
        if (T <= T_v1_to_v2) {
            vc = v1;
            T1 = 0;
            T2 = T;
        }
        else if (ve2 <= v1 && T <= T_v1_to_ve2 + T_v2_to_ve2 ) {
            //% 第一段无匀加速，第二段无匀加速
            //% clear
            //% syms T1 T2 T v1 v2 vc a j pt
            //% T1 = 2 * sqrt((v1 - vc) / j)
            //% T2 = 2 * sqrt((v2 - vc) / j)
            //% vc_ans = solve(T1 + T2 == T, vc)
            vc = (-T*T*j + 8*v1 + 8*v2)/16 - (v2-v1)/T * (v2-v1)/T /j;
            T2 = 2 * safe_sqrt((v2 - vc) / j);
            T1 = T - T2;
        }
        else if (T <= T_v1_to_ve1 + T_v2_to_ve1) {
            //% 第一段有匀加速，第二段无匀加速
            //% clear
            //% syms T1 T2 T v1 v2 vc a j pt
            //% T1 = 2 * sqrt((v1 - vc) / j)
            //% T2 = (v2 - vc) / a + a / j
            //% solve(T1 + T2 == T, vc)
            vc = (j * v2 - a*a + 2 * a * j * safe_sqrt((v1 - v2 + T * a) / j) - T * a * j) / j;
            T2 = (v2 - vc) / a + a / j;
            T1 = T - T2;
        }
        else {
            //% 第一段有匀加速，第二段有匀加速
            //% clear
            //% syms T1 T2 T v1 v2 vc a j pt
            //% T1 = (v1 - vc) / a + a / j
            //% T2 = (v2 - vc) / a + a / j
            //% solve(T1 + T2 == T, vc)
            vc = (2 * a*a - T * j * a + j * v1 + j * v2) / (2 * j);
            T2 = (v2 - vc) / a + a / j;
            T1 = T - T2;
        }

        //% 更新 vc_below, Ta, Tb
        *vc_below = vc;
        if (vb > va) {
            *Ta = T1;
            *Tb = T2;
        }
        else {
            *Ta = T2;
            *Tb = T1;
        }

        return *vc_below;
    }

    // 考虑 pt，且必定成功，因为在计算 T 的范围时，已经考虑了 vb 的可实现性
    auto s_scurve_cpt_vb_upper(const SCurveParam& param)-> double {
        //%
        //% 1. 确定必然可以达到的 vb_min vb_max vc_min vc_max
        //%
        //%2. 确定 vb_max 对应的最大可能的vc：vc_l（vb_min 对应 vc_s）
        //%
        //%3. 确定 vc_min 对应的最小可能vb：vb_s（vc_max 对应 vb_l）
        //%
        //%vc        vb           Ta         Tc       Tb         requires
        //% 1 vc_min   vb_min       T_va2vcmin T - Ta - Tb T_vcmin2vbmin none
        //% 2 vc_min   vc_min - a * a / j T_va2vcmin T - Ta - Tb 2 * a / j         T, vb
        //% 3 vc_min   vb_s         T_va2vcmin T - Ta - Tb T_vcmin2vbs   none
        //% 4 va / -Ta   vc - a * a / j     T - Tb       0       2 * a / j         T > 4 * a / j
        //% 5 va + a * a / j vc\ - Tb       2 * a / j      0       T - Ta          T > 4 * a / j
        //% 6 vc_l     vb_max        Ta        Tc         Tb         none
        //%
        //%l1->l3->l6 依次增加
        //%
        //%上式先计算 l1 l3 l6
        //% l2 若 vc_min + a * a / j < vb_min, 则 l2 = l1
        //% 若 vc_min + a * a / j > vb_max, 则 l2 = l3
        //% 否则 按照公式计算
        //%
        //%l4 按照公式计算
        //%
        //%l5 按照公式计算
        //%
        //%l1 <= pt      时：vb_upper = vb_min
        //% l2 <= pt < l1 时：有匀速段，Tb段无匀加速，计算Tb
        //% l3 <= pt < l2 时：有匀速段，Tb段有匀加速，计算Tb
        //% T > 4 * a / j 时：l4 > l5
        //% l4 <= pt < l3 时：无匀速段，Ta段有匀加速，Tb段无匀加速
        //% l5 <= pt < l4 时：无匀速段，Ta段有匀加速，Tb段有匀加速
        //% l6 <= pt < l5 时：无匀速段，Ta段无匀加速，Tb段有匀加速
        //% T <= 4 * a / j && T > 2 * a / j 时：l5 > l4
        //% l5 <= pt < l3 时：无匀速段，Ta段有匀加速，Tb段无匀加速
        //% l4 <= pt < l5 时：无匀速段，Ta段无匀加速，Tb段无匀加速
        //% l6 <= pt < l4 时：无匀速段，Ta段无匀加速，Tb段有匀加速
        //% T <= 2 * a / j 时：
        //% l6 <= pt < l3 时：无匀速段，Ta段无匀加速，Tb段无匀加速
        //% pt < l6 时：vb_upper = vb_max
        const double pt = param.pb_ - param.pa_;
        const double va = param.va_;
        const double a = param.a_;
        const double j = param.j_;
        const double T = param.T_;
        double vb_max = param.vb_max_;
        double vc_max = param.vc_max_;
        double vb_min = 0;
        double vc_min = 0;
        
        constexpr double cons = 1e-9;
        double Z1 = a*a / j;
        double Z2 = T*T * j;

        //% STEP 1: 确定必然可以达到的 vb_min vb_max vc_min vc_max
        vb_min = std::max(vb_min, s_acc_vend(va, -a, -j, T));
        vb_max = std::min(vb_max, s_acc_vend(va, a, j, T));
        vc_min = std::max(vc_min, s_scurve_cpt_vc_below_by_va_vb_T(va, vb_min, T, a, j));

        double T_va_to_vcmin = s_acc_time(va, vc_min, a, j);
        double T_vcmin_to_vbmin = s_acc_time(vc_min, vb_min, a, j);

        //% STEP 2: 确定 vb_max 对应的最大可能的vc：vc_s
        double vc_s = std::max(vc_min, s_scurve_cpt_vc_below_by_va_vb_T(va, vb_max, T, a, j));

        //% STEP 3: 确定 vc_min 对应的最达可能vb：vb_l
        double vb_l = std::min(vb_max, s_acc_vend(vc_min, a, j, T - T_va_to_vcmin));

        double vc, vb, Ta, Tb, Tc, l1, l2, l3, l4, l5, l6;
        //% CASE 1 ------------------pt < l1
        vc = vc_min;
        vb = vb_min;
        Ta = T_va_to_vcmin;
        Tb = T_vcmin_to_vbmin;
        Tc = T - Ta - Tb;
        l1 = (va + vc) * Ta / 2 + vc * Tc + (vc + vb) * Tb / 2;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        if (pt <= l1 - cons) {
            THROW_FILE_LINE("FAILED IN s_scurve_cpt_vb_upper");
            return -1;
        }
#endif

        if (pt <= l1)
            return vb_min;

        //% CASE 2 ------------------l1 < pt < l3
        vc = vc_min;
        vb = vb_l;
        Ta = T_va_to_vcmin;
        Tb = s_acc_time(vc, vb, a, j);
        Tc = T - Ta - Tb;
        l3 = (va + vc) * Ta / 2 + vc * Tc + (vc + vb) * Tb / 2;
        if (pt <= l3) {
            //% 计算 l2
            if (vc_min + a * a / j < vb_min)
                l2 = l1;
            else if (vc_min + a * a / j > vb_l)
                l2 = l3;
            else {
                vc = vc_min;
                vb = vc_min - a * a / j;
                Ta = T_va_to_vcmin;
                Tb = s_acc_time(vc, vb, a, j);
                Tc = T - Ta - Tb;
                l2 = (va + vc) * Ta / 2 + vc * Tc + (vc + vb) * Tb / 2;
            }

            if (pt <= l2) {
                //% clear
                //% syms va j Ta a T pt Tb la vc vc_min
                //% vb = vc_min + j * Tb * Tb / 4
                //%
                //%l = Tb * (vb + vc_min) / 2 + la + (T - Ta - Tb) * vc_min;
                //% collect(l, Tb)
                //% solve(l == pt, Tb)
                Ta = T_va_to_vcmin;
                double la = Ta * (va + vc_min) / 2;
                Tb = std::cbrt(-(8 * (la - pt + T * vc_min - Ta * vc_min)) / j);
                vb = vc_min + j * Tb * Tb / 4;

                return vb;
            }
            else {
                //% clear
                //% syms Ta la va vc_min pt a j Tb T
                //% vb = vc_min + a * Tb - a * a / j
                //% l = Tb * (vb + vc_min) / 2 + (T - Ta - Tb) * vc_min + la;
                //% collect(l, Tb)
                //% solve(l == pt, Tb)
                Ta = T_va_to_vcmin;
                double la = Ta * (va + vc_min) / 2;
                double B = -a / j;
                double C = (2 * la - 2 * pt + 2 * vc_min * (T - Ta)) / a;

                Tb = (-B + safe_sqrt(std::max(B * B - 4 * C, 0.0))) / 2;
                vb = vc_min + Tb * a - Z1;
                return vb;
            }
        }
            
        //% CASE 3 ------------------l3 < pt < l6
        vc = vc_s;
        vb = vb_max;
        Ta = s_acc_time(va, vc, a, j);
        Tb = s_acc_time(vc, vb, a, j);
        Tc = T - Ta - Tb;
        l6 = (va + vc) * Ta / 2 + vc * Tc + (vc + vb) * Tb / 2;
        if (pt <= l6) {
            Tb = 2 * a / j;
            Ta = T - Tb;
            vc = s_acc_vend(va, -a, -j, Ta);
            vb = s_acc_vend(vc, a, j, Tb);
            l4 = (va + vc) * Ta / 2 + (vc + vb) * Tb / 2;

            Ta = 2 * a / j;
            Tb = T - Ta;
            vc = s_acc_vend(va, -a, -j, Ta);
            vb = s_acc_vend(vc, a, j, Tb);
            l5 = (va + vc) * Ta / 2 + (vc + vb) * Tb / 2;

            //% Ta无匀加速, Tb无匀加速
            if ((T <= 2 * a / j) || (2 * a / j < T && T <= 4 * a / j && l4 >= pt && pt > l5)) {
                //% syms a j T va Ta Tb pt
                //% vc = va - j * Ta * Ta / 4
                //% la = Ta * (va + vc) / 2
                //% Tb = T - Ta
                //% vb = vc + j * Tb * Tb / 4
                //% lb = Tb * (vc + vb) / 2
                //% collect(la + lb - pt, Ta)
                //% 【result】:
                //% 带入方程la + lb = pt
                //% 可得：
                //% k2 * Ta ^ 2 + k1 * Ta + k0
                //% 其中：
                //% k2 = T * j / 8
                //% k1 = -3 * T ^ 2 * j / 8
                //% k0 = -pt + (T * (2 * va + (T ^ 2 * j) / 4)) / 2
                //%
                //%Ta = (-k1 + sqrt(k1 * k1 - 4 * k0 * k2)) / 2 / k2
                double k2 = T * j / 8;
                double k1 = -3 * T * T * j / 8;
                double k0 = -pt + (T * (2 * va + (T * T * j) / 4)) / 2;

                //% 选根
                //% 其极值为(k1) / (2 * k2) = (3 * T) / 2
                //% 因此需选其较小的根
                Ta = (-k1 - safe_sqrt(k1 * k1 - 4 * k0 * k2)) / 2 / k2;

                vc = va - j * Ta * Ta / 4;
                Tb = T - Ta;
                vb = vc + j * Tb * Tb / 4;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //% debug check%
                double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
                if (vc < vc_min - cons || vc > vc_max + cons || std::abs(l - pt) > cons || std::abs(Ta + Tb - T) > cons) {
                    THROW_FILE_LINE("wrong vb_upper in CASE 2.1");
                }
#endif
                return vb;
            }

            //% Ta有匀加速, Tb无匀加速
            if ((4 * a / j < T && l4 >= pt && pt > l3) ||
                (2 * a / j < T && T <= 4 * a / j && l5 >= pt && pt > l3))
            {
                //% clear
                //% syms va T a j pt Ta
                //% vc = va - Ta * a + a ^ 2 / j
                //% la = Ta * (va + vc) / 2
                //% Tb = T - Ta
                //% vb = vc + j * Tb * Tb / 4;
                //% lb = Tb * (vc + vb) / 2
                //% collect(la + lb - pt, Ta)
                //%
                //%【result】:
                //% 带入方程la + lb = pt
                //% 可得：
                //% k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% 其中：
                //% k3 = -j / 8
                //% k2 = (a / 2 + (3 * T * j) / 8)
                //% k1 = (-(T ^ 2 * j) / 8 - a ^ 2 / (2 * j) - (T * (2 * a + (T * j) / 2)) / 2)
                //% k0 = -pt + (T * (2 * va + (T ^ 2 * j) / 4 + (2 * a ^ 2) / j)) / 2
                //%
                //%【condition】:
                //% Tb > 2 * a / j
                //% = > Ta < T - 2 * a / j
                //% 于是：
                //% 0 <= Ta <= min(T, 2 * a / j, T - 2 * a / j)
                //%
                //%计算Ta = min(T, 2 * a / j, T - 2 * a / j)
                //% l3 = la + lb
                double k3 = -j;
                double k2 = 4*a + 3*T*j;
                double k1 = -3*j*T*T - 8*a*T - 4*a*a/j;
                double k0 = j*T*T*T + 8*T*a*a/j + 8*va*T - 8*pt;

                //% 选根
                //% syms f(Ta) g(Ta)
                //% f(Ta) = k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% g(Ta) = diff(f, Ta)
                //% solve(g, Ta)
                //%
                //%可得其极值：
                //% r1 = (2*a + T*j)/j
                //% r2 = (2*a + 3*T*j)/(3*j)
                //%
                //% 均大于T，因此其上下界为[0, T]
                Ta = newton_raphson_binary_search([k3, k2, k1, k0](double x){
                    return ((k3 * x + k2) * x + k1) * x + k0;
                    }, 0, T_va_to_vcmin);

                vc = va - Ta * a + a * a / j;
                Tb = T - Ta;
                vb = vc + j * Tb * Tb / 4.0;
                double vb_upper = vb;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //% debug check%
                double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
                if (vc < vc_min - cons || vc > vc_max + cons || std::abs(l - pt) > std::max(pt,1.0)*cons || std::abs(Ta + Tb - T) > cons) {
                    THROW_FILE_LINE("wrong vb_upper in CASE 2.3");
                }
#endif
                return vb_upper;
            }

            //% Ta无匀加速, Tb有匀加速
            if ((4 * a / j < T && l6 >= pt && pt > l5) ||
                (2 * a / j < T && T <= 4 * a / j && l6 >= pt && pt > l4)) 
            {
                //% clear
                //% syms va j Ta a T vc pt
                //% vc = va - j * Ta * Ta / 4
                //% la = Ta * (va + vc) / 2
                //% Tb = T - Ta
                //% vb = vc + Tb * a - a ^ 2 / j
                //% lb = Tb * (vc + vb) / 2
                //% l = la + lb
                //% collect(la + lb - pt, Ta)
                //%
                //%【result】:
                //% 带入方程la + lb = pt
                //% 可得：
                //% k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% 其中：
                //% k3 = j / 8
                //% k2 = (a / 2 - (T * j) / 4)
                //% k1 = (a ^ 2 / (2 * j) - T * a)
                //% k0 = -pt + (T * (2 * va + T * a - a ^ 2 / j)) / 2
                //%
                //%【condition】:
                //% Tb > 2 * a / j
                //% = > Ta < T - 2 * a / j
                //% 于是：
                //% 0 <= Ta <= min(T, 2 * a / j, T - 2 * a / j)
                //%
                //%计算Ta = min(T, 2 * a / j, T - 2 * a / j)
                //% l3 = la + lb
                //double k3 = j / 8;
                //double k2 = (a / 2 - (T * j) / 4);
                //double k1 = (a*a / (2 * j) - T * a);
                //double k0 = -pt + (T * (2 * va + T * a - a*a / j)) / 2;

                double k3 = j;
                double k2 = 4*a - 2*T*j;
                double k1 = -8*a*T + 4*a*a/j;
                double k0 = 4*a*T*T - 4*T*a*a/j + 8*va*T - 8 * pt;

                //% 选根
                //% syms f(Ta) g(Ta)
                //% f(Ta) = k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% g(Ta) = diff(f, Ta)
                //% solve(g, Ta)
                //%
                //%可得其极值：
                //% r1 = 4 / 3 * T - 2 / 3 * a / j
                //% r2 = -(2 * a) / j
                //%
                //%由于b段可达最大加速度，因此必有 T >= 2a / j
                //% 于是
                //% r1 >= 4 / 3 * T - 1 / 3 * T = T
                //% 因此其上下界为[0, T]
                Ta = newton_raphson_binary_search([k3, k2, k1, k0](double x) {
                    return ((k3 * x + k2) * x + k1) * x + k0;
                    }, 0, T_va_to_vcmin);

                vc = va - j * Ta * Ta / 4;
                Tb = T - Ta;
                vb = s_acc_vend(vc, a, j, Tb);
                double vb_upper = vb;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //% debug check%
                double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
                if (vc < vc_min - cons || vc > vc_max + cons || std::abs(l - pt) > cons || std::abs(Ta + Tb - T) > cons) {
                    throw std::runtime_error("wrong vb_upper in CASE 2.2");
                }
#endif 
                return vb_upper;
            }

            //% Ta有匀加速, Tb有匀加速
            if ((4 * a / j < T && l5 >= pt && pt > l4)) {
                //% syms va j Ta a T pt
                //% vc = va - Ta * a + a * a / j
                //% la = Ta * (va + vc) / 2
                //% Tb = T - Ta
                //% vb = vc + Tb * a - a * a / j
                //% lb = Tb * (vc + vb) / 2;
                //%
                //% 根据 la + lb = pt，有：
                //% collect(la + lb - pt, Ta)
                //%
                //%可得方程系数
                double k2 = a;
                double k1 = -2 * T * a;
                double k0 = -pt + (T * (2 * va + T * a + a*a / j)) / 2;

                Ta = (-k1 - safe_sqrt(k1 * k1 - 4 * k2 * k0)) / (2 * k2);

                vc = va - Ta * a + a * a / j;
                Tb = std::max(T - Ta, 0.0);
                vb = s_acc_vend(vc, a, j, Tb);
                double vb_upper = s_acc_vend(vc, a, j, Tb);
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //% debug check%
                double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
                if (vc < vc_min - cons || vc > vc_max + cons || std::abs(l - pt) > cons || std::abs(Ta + Tb - T) > cons) {
                    THROW_FILE_LINE("wrong vb_upper in CASE 2.4");
                }
#endif
                return vb_upper;
            }
        }


        //% CASE 4 ------------------l6 < pt
        if (pt > l6) {
            double vb_upper = vb_max;
            return vb_upper;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        else
        {
            THROW_FILE_LINE("condition check failed");
            return vb_max;
        }
#endif
        return vb_max;
        
    }
    auto s_scurve_cpt_vb_below(const SCurveParam& param) -> double {
        //%
        //% 1. 确定必然可以达到的 vb_max vb_min vc_max vc_min
        //%
        //%2. 确定 vb_min 对应的最大可能的vc：vc_l（vb_max 对应 vc_s）
        //%
        //%3. 确定 vc_max 对应的最小可能vb：vb_s（vc_min 对应 vb_l）
        //%
        //%vc        vb           Ta         Tc       Tb         requires
        //% 1 vc_max   vb_max       T_va2vcmax T - Ta - Tb T_vcmax2vbmax none
        //% 2 vc_max   vc_max - a * a / j T_va2vcmax T - Ta - Tb 2 * a / j         T, vb
        //% 3 vc_max   vb_s         T_va2vcmax T - Ta - Tb T_vcmax2vbs   none
        //% 4 va / -Ta   vc - a * a / j     T - Tb       0       2 * a / j         T > 4 * a / j
        //% 5 va + a * a / j vc\ - Tb       2 * a / j      0       T - Ta          T > 4 * a / j
        //% 6 vc_l     vb_min        Ta        Tc         Tb         none
        //%
        //%l1->l3->l6 依次减小
        //%
        //%上式先计算 l1 l3 l6
        //% l2 若 vc_max - a * a / j > vb_max, 则 l2 = l1
        //% 若 vc_max - a * a / j < vb_min, 则 l2 = l3
        //% 否则 按照公式计算
        //%
        //%l4 按照公式计算
        //%
        //%l5 按照公式计算
        //%
        //%l1 <= pt      时：vb_upper = vb_max
        //% l2 <= pt < l1 时：有匀速段，Tb段无匀加速，计算Tb
        //% l3 <= pt < l2 时：有匀速段，Tb段有匀加速，计算Tb
        //% T > 4 * a / j 时：l4 > l5
        //% l4 <= pt < l3 时：无匀速段，Ta段有匀加速，Tb段无匀加速
        //% l5 <= pt < l4 时：无匀速段，Ta段有匀加速，Tb段有匀加速
        //% l6 <= pt < l5 时：无匀速段，Ta段无匀加速，Tb段有匀加速
        //% T <= 4 * a / j && T > 2 * a / j 时：l5 > l4
        //% l5 <= pt < l3 时：无匀速段，Ta段有匀加速，Tb段无匀加速
        //% l4 <= pt < l5 时：无匀速段，Ta段无匀加速，Tb段无匀加速
        //% l6 <= pt < l4 时：无匀速段，Ta段无匀加速，Tb段有匀加速
        //% T <= 2 * a / j 时：
        //% l6 <= pt < l3 时：无匀速段，Ta段无匀加速，Tb段无匀加速
        //% pt < l6 时：vb_upper = vb_min
        const double pt = param.pb_ - param.pa_;
        const double va = param.va_;
        const double a = param.a_;
        const double j = param.j_;
        const double T = param.T_;
        double vb_max = param.vb_max_;
        double vc_max = param.vc_max_;
        double vb_min = 0;

        constexpr double cons = 1e-9;
        double Z1 = a * a / j;
        double Z2 = T * T * j;

        //% STEP 1: 确定必然可以达到的 vb_max vb_min vc_max vc_min
        vb_max = std::min(vb_max, s_acc_vend(va, a, j, T));
        vb_min = std::max(vb_min, s_acc_vend(va, -a, -j, T));
        vc_max = std::min(vc_max, s_scurve_cpt_vc_upper_by_va_vb_T(va, vb_max, T, a, j));

        double T_va_to_vcmax = s_acc_time(va, vc_max, a, j);
        double T_vcmax_to_vbmax = s_acc_time(vc_max, vb_max, a, j);

        //% STEP 2: 确定 vb_min 对应的最大可能的vc：vc_l
        double vc_l = std::min(vc_max, s_scurve_cpt_vc_upper_by_va_vb_T(va, vb_min, T, a, j));

        //% STEP 3: 确定 vc_max 对应的最小可能vb：vb_s
        double vb_s = std::max(vb_min, s_acc_vend(vc_max, -a, -j, T - T_va_to_vcmax));

        double vc, vb, Ta, Tb, Tc, l1, l2, l3, l4, l5, l6;
        //% CASE 1 ------------------pt > l1
        vc = vc_max;
        vb = vb_max;
        Ta = T_va_to_vcmax;
        Tb = T_vcmax_to_vbmax;
        Tc = T - Ta - Tb;
        l1 = (va + vc) * Ta / 2 + vc * Tc + (vc + vb) * Tb / 2;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        if (pt >= l1 + 1e-10) {
            THROW_FILE_LINE("FAILED IN s_scurve_cpt_vb_upper");
            return std::numeric_limits<double>::infinity();
        }
            
#endif

        if (pt >= l1)
            return vb_max;

        //% CASE 2 ------------------l3 < pt < l1
        vc = vc_max;
        vb = vb_s;
        Ta = T_va_to_vcmax;
        Tb = s_acc_time(vc, vb, a, j);
        Tc = T - Ta - Tb;
        l3 = (va + vc) * Ta / 2 + vc * Tc + (vc + vb) * Tb / 2;
        if (pt >= l3) {
            if (vc_max - a * a / j > vb_max)
                l2 = l1;
            else if (vc_max - a * a / j < vb_s)
                l2 = l3;
            else {
                vc = vc_max;
                vb = vc_max - a * a / j;
                Ta = T_va_to_vcmax;
                Tb = s_acc_time(vc, vb, a, j);
                Tc = T - Ta - Tb;
                l2 = (va + vc) * Ta / 2 + vc * Tc + (vc + vb) * Tb / 2;
            }

            if (pt >= l2) {
                //% clear
                //% syms la vc_max pt Ta Tb T
                //% vb = vc_max - j * Tb * Tb / 4;
                //% l = la + (T - Ta - Tb) * vc_max + (vc_max + vb) / 2 * Tb
                //% solve(l == pt, Tb)
                Ta = T_va_to_vcmax;
                double la = Ta * (va + vc_max) / 2;
                Tb = std::cbrt((la + vc_max * (T - Ta) - pt) * 8 / j);
                vb = vc_max - j * Tb * Tb / 4;
                return vb;
            }
            else {
                Ta = T_va_to_vcmax;
                double la = Ta * (va + vc_max) / 2;
                double B = -a / j;
                double C = -(2 * la - 2 * pt + 2 * vc_max * (T - Ta)) / a;

                Tb = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                vb = vc_max - Tb * a + Z1;

                return vb;
            }
        }
            
        //% CASE 3 ------------------l6 < pt < l3
        vc = vc_l;
        vb = vb_min;
        Ta = s_acc_time(va, vc, a, j);
        Tb = s_acc_time(vc, vb, a, j);
        Tc = T - Ta - Tb;
        l6 = (va + vc) * Ta / 2 + vc * Tc + (vc + vb) * Tb / 2;
        if (pt >= l6) {
            Tb = 2 * a / j;
            Ta = T - Tb;
            vc = s_acc_vend(va, a, j, Ta);
            vb = s_acc_vend(vc, -a, -j, Tb);
            l4 = (va + vc) * Ta / 2 + (vc + vb) * Tb / 2;

            Ta = 2 * a / j;
            Tb = T - Ta;
            vc = s_acc_vend(va, a, j, Ta);
            vb = s_acc_vend(vc, -a, -j, Tb);
            l5 = (va + vc) * Ta / 2 + (vc + vb) * Tb / 2;

            //% Ta无匀加速, Tb无匀加速
            if ((T <= 2 * a / j) || (2 * a / j < T && T <= 4 * a / j && l4 <= pt && pt < l5)) {
                //% clear
                //% syms va j Ta a T pt
                //% vc = va + j * Ta * Ta / 4
                //% la = Ta * (va + vc) / 2
                //% Tb = T - Ta
                //% vb = vc - j * Tb * Tb / 4
                //% lb = Tb * (vc + vb) / 2
                //% collect(la + lb - pt, Ta)
                //% 【result】:
                //% 带入方程la + lb = pt
                //% 可得：
                //% k2 * Ta ^ 2 + k1 * Ta + k0
                //% 其中：
                //% k2 = T * j / 8
                //% k1 = -3 * T ^ 2 * j / 8
                //% k0 = pt - (T * (2 * va - (T ^ 2 * j) / 4)) / 2
                //%
                //%Ta = (-k1 + sqrt(k1 * k1 - 4 * k0 * k2)) / 2 / k2
                double k2 = T * j / 8;
                double k1 = -3 * T * T * j / 8;
                double k0 = pt - (T * (2 * va - T * T * j / 4)) / 2;

                //% 选根
                //% 其极值为(k1) / (2 * k2) = (3 * T) / 2
                //% 因此需选其较小的根
                Ta = (-k1 - safe_sqrt(k1 * k1 - 4 * k0 * k2)) / 2 / k2;

                vc = va + j * Ta * Ta / 4;
                Tb = T - Ta;
                vb = vc - j * Tb * Tb / 4;
                double vb_below = vb;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //% debug check%
                double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
                if (vc < -cons || vc > vc_max + cons || std::abs(l - pt) > cons || std::abs(Ta + Tb - T) > cons) {
                    THROW_FILE_LINE("wrong vb_below in CASE 2.1");
                }
#endif
                return vb_below;
            }

            //% Ta有匀加速, Tb无匀加速
            if ((4 * a / j < T && l4 <= pt && pt < l3) ||
                (2 * a / j < T && T <= 4 * a / j && l5 <= pt && pt < l3)) 
            {
                //% clear
                //% syms va T a j pt Ta
                //% vc = va + Ta * a - a ^ 2 / j
                //% la = Ta * (va + vc) / 2
                //% Tb = T - Ta
                //% vb = vc - j * Tb * Tb / 4;
                //% lb = Tb * (vc + vb) / 2
                //% collect(la + lb - pt, Ta)
                //%
                //%【result】:
                //% 带入方程la + lb = pt
                //% 可得：
                //% k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% 其中：
                //% k3 = j / 8
                //% k2 = (-a / 2 - (3 * T * j) / 8)
                //% k1 = ((T ^ 2 * j) / 8 + a ^ 2 / (2 * j) + (T * (2 * a + (T * j) / 2)) / 2)
                //% k0 = -pt - (T * ((T ^ 2 * j) / 4 - 2 * va + (2 * a ^ 2) / j)) / 2
                //%
                //%
                //%【condition】:
                //% Tb > 2 * a / j
                //% = > Ta < T - 2 * a / j
                //% 于是：
                //% 0 <= Ta <= min(T, 2 * a / j, T - 2 * a / j)
                //%
                //%计算Ta = min(T, 2 * a / j, T - 2 * a / j)
                //% l3 = la + lb
                double k3 = j / 8;
                double k2 = (-a / 2 - (3 * T * j) / 8);
                double k1 = ((T*T * j) / 8 + a*a / (2 * j) + (T * (2 * a + (T * j) / 2)) / 2);
                double k0 = -pt - (T * ((T*T * j) / 4 - 2 * va + (2 * a*a) / j)) / 2;

                //% 选根
                //% syms f(Ta) g(Ta)
                //% f(Ta) = k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% g(Ta) = diff(f, Ta)
                //% solve(g, Ta)
                //%
                //%可得其极值：
                //% r1 = 4 / 3 * T - 2 / 3 * a / j
                //% r2 = -(2 * a) / j
                //%
                //%由于b段可达最大加速度，因此必有 T >= 2a / j
                //% 于是
                //% r1 >= 4 / 3 * T - 1 / 3 * T = T
                //% 因此其上下界为[0, T]
                Ta = newton_raphson_binary_search([k3, k2, k1, k0](double x) {
                    return ((k3 * x + k2) * x + k1) * x + k0;
                    }, 0, T_va_to_vcmax);

                vc = va + Ta * a - Z1;
                Tb = T - Ta;
                vb = s_acc_vend(vc, -a, -j, Tb);
                double vb_below = vb;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //% debug check%
                double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
                if (vc < -cons || vc > vc_max + cons || std::abs(l - pt) > std::max(pt, 1.0) * cons || std::abs(Ta + Tb - T) > cons) {
                    THROW_FILE_LINE("wrong vb_below in CASE 2.3");
                }
#endif
                return vb_below;
            }

            //% Ta无匀加速, Tb有匀加速
            if ((4 * a / j < T && l6 <= pt && pt < l5) ||
                (2 * a / j < T && T <= 4 * a / j && l6 <= pt && pt < l4)) 
            {
                //% clear
                //% syms va j Ta a T
                //% vc = va + j * Ta * Ta / 4
                //% la = Ta * (vc + va) / 2
                //% Tb = T - Ta
                //% vb = vc - Tb * a + a ^ 2 / j;
                //% lb = Tb * (vc + vb) / 2
                //% l = la + lb
                //%
                //%【result】:
                //% 带入方程la + lb = pt
                //% 可得：
                //% k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% 其中：
                //% k3 = -j / 8
                //% k2 = (T * j) / 4 - a / 2
                //% k1 = -a ^ 2 / (2 * j) + T * a
                //% k0 = (T * (a ^ 2 / j - T * a + 2 * va)) / 2 - pt
                //%
                //%【condition】:
                //% Tb > 2 * a / j
                //% = > Ta < T - 2 * a / j
                //% 于是：
                //% 0 <= Ta <= min(T, 2 * a / j, T - 2 * a / j)
                //%
                //%计算Ta = min(T, 2 * a / j, T - 2 * a / j)
                //% l3 = la + lb
                double k3 = -j / 8;
                double k2 = (T * j) / 4 - a / 2;
                double k1 = -Z1 / 2 + T * a;
                double k0 = (T * (Z1 - T * a + 2 * va)) / 2 - pt;

                //% 选根
                //% syms f(Ta) g(Ta)
                //% f(Ta) = k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% g(Ta) = diff(f, Ta)
                //% solve(g, Ta)
                //%
                //%可得其极值：
                //% r1 = 4 / 3 * T - 2 / 3 * a / j
                //% r2 = -(2 * a) / j
                //%
                //%由于b段可达最大加速度，因此必有 T >= 2a / j
                //% 于是
                //% r1 >= 4 / 3 * T - 1 / 3 * T = T
                //% 因此其上下界为[0, T]
                Ta = newton_raphson_binary_search([k3, k2, k1, k0](double x) {
                    return ((k3 * x + k2) * x + k1) * x + k0;
                    }, 0, T_va_to_vcmax);

                vc = va + j * Ta * Ta / 4;
                Tb = T - Ta;
                vb = s_acc_vend(vc, -a, -j, Tb);
                double vb_below = vb;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //% debug check%
                double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
                if (vc < -cons || vc > vc_max + cons || std::abs(l - pt) > cons || std::abs(Ta + Tb - T) > cons) {
                    THROW_FILE_LINE("wrong vb_below in CASE 2.2");
                }   
#endif
                return vb_below;
            }

            //% Ta有匀加速, Tb有匀加速    
            if ((4 * a / j < T && l5 <= pt && pt < l4)) {
                //% syms va T a j pt Ta
                //% vc = va + Ta * a - a ^ 2 / j
                //% la = Ta * (va + vc) / 2
                //% Tb = T - Ta
                //% vb = vc - Tb * a + a ^ 2 / j;
                //% lb = Tb * (vc + vb) / 2
                //%
                //%根据 la + lb = pt，有：
                //% collect(la + lb - pt, Ta)
                //% -a * Ta ^ 2 + (2 * T * a + a ^ 2 / j) * Ta - pt - (T * (T * a - 2 * va + (3 * a ^ 2) / j)) / 2
                //%
                //%可得方程系数
                double k2 = -a;
                double k1 = 2 * T * a;
                double k0 = -pt - (T * (T * a - 2 * va + a*a / j)) / 2;

                //% 选根
                //% syms f(Ta) g(Ta)
                //% f(Ta) = k3 * Ta ^ 3 + k2 * Ta ^ 2 + k1 * Ta + k0
                //% g(Ta) = diff(f, Ta)
                //% solve(g, Ta)
                //%
                //%得到:
                //%
                //%r1 = T + (2 * a) / (3 * j)
                //% r2 = T + (2 * a) / j
                //%
                //%因为必有 2 * a / j <= Ta <= T, 因此其上下界为：
                //% T_below = 2 * a / j
                //% T_upper = T

                Ta = (-k1 + safe_sqrt(k1 * k1 - 4 * k2 * k0)) / (2 * k2);

                vc = va + Ta * a - Z1;
                Tb = std::max(T - Ta, 0.0);
                vb = s_acc_vend(vc, -a, -j, Tb);
                double vb_below = vb;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //% debug check%
                double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
                if (vc < -cons || vc > vc_max + cons || std::abs(l - pt) > cons || std::abs(Ta + Tb - T) > cons) {
                    THROW_FILE_LINE("wrong vb_upper in CASE 2.4");
                }
#endif
                return vb_below;
            }
        }

        //% CASE 4 ------------------pt < l6
        if (pt < l6) {
            return vb_min;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        else {
            THROW_FILE_LINE("condition check failed");
            return vb_min;
        }
#endif
        return vb_min;
    }
    auto s_scurve_cpt_vb_range(SCurveParam& param) ->std::tuple<double, double> {
        const double cons = 1e-10;
        
        const double pa = param.pa_;
        const double pb = param.pb_;
        const double pt = param.pb_ - param.pa_;
        const double va = param.va_;
        const double a = param.a_;
        const double j = param.j_;
        const double T = param.T_;

        double vb_max = param.vb_max_;
        double vc_max = param.vc_max_;
        double vc_min = 0;
        double vb_min = 0;

        //% 计算恰好可以到达 pt T 的 v1 v2
        double v_diff = s_acc_vend(0, a, j, T);
        double v_dis = pt / T - v_diff / 2;

        double v1_below = v_dis;
        double v2_upper = v_diff + v_dis;

        //% 整个过程的速度应该在上述 v1 v2 之间，因此调整 vc 的范围
        vc_min = std::max(vc_min, v1_below);
        vc_max = std::min(vc_max, v2_upper);

        //% 但是上述 vc_min, vc_max 仍然未必可达，还需再考虑限制
        double v_avg = pt / T;
        if (std::abs(vc_min - v_avg) < std::abs(vc_max - v_avg)) {
            //% vc_max 可能无法达到，vc_min 一定可以达到
            param.va_ = vc_min;
            
            double vb_max_ori = param.vb_max_;
            param.vb_max_ = vc_max;
            vc_max = s_scurve_cpt_vb_upper(param);
            param.vb_max_ = vb_max_ori;
        }   
        else {
            //% vc_min 可能无法达到
            param.va_ = vc_max;
            vc_min = s_scurve_cpt_vb_below(param);
        }

        double va_upper_ori_ = param.va_upper_;
        double va_below_ori_ = param.va_below_;

        //% 修正 va_range
        param.va_upper_ = std::min(param.va_upper_, vc_max);
        param.va_below_ = std::max(param.va_below_, vc_min);

        //% 计算 vb_range
        param.va_ = param.va_below_;
        param.vb_upper_ = s_scurve_cpt_vb_upper(param);
        param.va_ = param.va_upper_;
        param.vb_below_ = s_scurve_cpt_vb_below(param);

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        if (param.vb_below_ > param.vb_upper_ + cons) {
            param.va_ = param.va_below_;
            param.vb_upper_ = s_scurve_cpt_vb_upper(param);
            param.va_ = param.va_upper_;
            param.vb_below_ = s_scurve_cpt_vb_below(param);
            THROW_FILE_LINE("failed in vb range");
        }
#endif
        //% 根据 vb_max 进行修正
        param.vb_upper_ = std::min(vb_max, param.vb_upper_);
        param.vb_upper_ = std::max(vb_min, param.vb_upper_);
        param.vb_below_ = std::max(vb_min, param.vb_below_);
        param.vb_below_ = std::min(vb_max, param.vb_below_);

        param.va_upper_ = va_upper_ori_;
        param.va_below_ = va_below_ori_;

        return std::make_tuple(param.vb_upper_, param.vb_below_);
    }

    // T_upper 与 T_below 必定会成功，T_range可能会失败
    auto s_scurve_cpt_T_upper(const SCurveParam& param, TRange *range, int *range_num) -> double {
        const double va = param.va_;
        const double vb_max = param.vb_max_;
        const double a = param.a_;
        const double j = param.j_;
        const double pt = param.pb_ - param.pa_;
        
        const double Z1 = a * a / j;
        std::vector<TRange> ret;

        //% 计算vb
        //  因为可能有多解需要计算出距离 vb_max 最近的 vb 值
        // 
        // 1）在 vb >= va - a^2/j 时，此时没有匀加速段
        //% 此时无匀加速段
        //% 前进时间为：t = 2 * sqrt( (va-vb) / j );
        //% 前进长度的平方为：
        //% p^2 = (t * (va+vb)/2)^2
        //%     =  va^3/j + (va^2*vb)/j - (va*vb^2)/j - vb^3/j
        //% vb 需求解一元三次方程 【1,va,-va^2,pb-pa-va^3】
        //%
        //% vb 可能有两个解，分别位于 【0,va/3】与【va/3，va】中
        // 
        // 1.1）va/3 <= va - a^2/j
        //   l 随 vb 在[va/3，va]单调递减，极大值位于 vb = va/3 处，极小值位于 vb = va处 
        //   考虑 vb 应当尽量贴近 vb_max，因此不考虑 vb < va/3 时的根
        // 
        // 1.2）va/3 > va - a^2/j
        //   l 随 vb 在[va - a^2/j，va]单调递减，极大值位于 vb = va - a^2/j 处，极小值位于 vb = va处
        // 
        // 2）在 vb <  va - a^2/j 时，此时拥有匀加速段
        //% 此时有匀速段
        //% 前进时间为：t = (va-vb)/a+a/j;
        //% 前进长度为：
        //% l = t*(va+vb)/2
        //%   = - vb^2/(2*a) + (a*vb)/(2*j) + (va*(a/j + va/a))/2
        //% vb 需求解一元二次方程 【
        //%       -1/(2*a),
        //%       a/(2*j),
        //%       (va*(a/j + va/a))/2-pt
        //% 】
        //% 对于根来说，应当取大值，这是因为Tmax应该尽可能的小
        // 其极值应当位于 a^2/(2*j) 处
        // 
        // 2.1）a^2/(2*j) <= va - a^2/j
        //   l 随 vb 在[a^2/(2*j)，va - a^2/j]单调递减，极大值位于 vb = a^2/(2*j) 处，极小值位于 vb = va - a^2/j 处 
        //   考虑 vb 应当尽量贴近 vb_max，因此不考虑 vb < a^2/(2*j) 时的根
        // 
        // 2.2）a^2/(2*j) >  va - a^2/j
        //   l 随 vb 在[0, va - a^2/j]单调递增，极大值位于 vb = va - a^2/j 处，极小值位于 vb = 0处
        // 
        //% 【条件1】 加速度正好可以达到a时，所前进的长度
        //% 此时 vb = - a^2/j + va
        //% 前进时间 t = (va-vb)/a+a/j
        //% 前进长度 l = t*(va+vb)/2 = (2*a*va)/j - a*a*a/j/j
        

        double T_va_to_0 = s_acc_time(va, 0, a, j);
        double l_va_to_0 = T_va_to_0 * va / 2;

        double T_va_to_vbmax = s_acc_time(va, vb_max, a, j);
        double l_va_to_vbmax = T_va_to_vbmax * (va + vb_max) / 2;

        double vk = va - a * a / j;
        double T_va_to_vk = s_acc_time(va, vk, a, j);
        double l_va_to_vk = T_va_to_vk * (va + vk) / 2;

        // v_peak at case 1
        double vp1 = va / 3;
        double T_va_to_vp1 = s_acc_time(va, vp1, a, j);
        double l_va_to_vp1 = T_va_to_vp1 * (va + vp1) / 2;

        // v_peak at case 2
        double vp2 = a*a / (2 * j);
        double T_va_to_vp2 = s_acc_time(va, vp2, a, j);
        double l_va_to_vp2 = T_va_to_vp2 * (va + vp2) / 2;

        // vb 的三个特殊值
        // vk : va - a*a/j, 为是否有匀减速的分界点
        // vp1: va / 3, 没有匀减速时，l取极值时的 vb 值
        // vp2: a*a/j/2 有匀减速时，l取极值时的vb
        //
        // 判断vk 与 vp1 vp2的关系
        // vk = 3*vp1 - 2*vp2
        // 
        // vk - vp1 = 2*（vp1-vp2）
        // vk - vp2 = 3*（vp1-vp2）
        //
        // 若   vp1 >= vp2
        // 则   vk >= vp1 >= vp2
        // 否则 vk < vp1 < vp2
        //
        // 因此至多有2个根
        //
        // CASE 1 vp1 >= vp2:
        //    CASE 1.0     l_vp1 <= pt
        //          T取值为：[Tmin，inf]
        //    CASE 1.1     l_vp1 > pt && pt <= l_0
        //          T取值为：[Tmin，s1] 并 [s2，inf]
        //    CASE 1.2     pt < l_0
        //          T取值为：[Tmin，s1]
        // CASE 2 vp < vp1:
        //    CASE 1.0     l_vp2 <= pt
        //          T取值为：[Tmin，inf]
        //    CASE 1.1     l_vp2 > pt && pt <= l_0
        //          T取值为：[Tmin，s1] 并 [s2，inf]
        //    CASE 1.2     pt < l_0
        //          T取值为：[Tmin，s1]



        double vb_solution[2];
        double vb_solution_num;
        if (vp1 >= vp2) {
            // 此时 vk >= vp1 >= vp2，可以达到 vp2
            double T_va_to_vp2 = s_acc_time(va, vp2, a, j);
            double l_va_to_vp2 = T_va_to_vp2 * (va + vp2) / 2;

            if (l_va_to_vp2 <= pt) {
                vb_solution_num = 0;
            }
            else if (l_va_to_0 <= pt) {
                vb_solution_num = 2;
                if (l_va_to_vk > pt) {
                    vb_solution[0] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                    , vk, va);
                }
                else {
                    double B = -Z1;
                    double C = 2 * pt * a - va * Z1 - va * va;
                    vb_solution[0] = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                }

                double B = -Z1;
                double C = 2 * pt * a - va * Z1 - va * va;
                vb_solution[1] = (-B - safe_sqrt(B * B - 4 * C)) / 2;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                if (std::abs(s_acc_time(va, vb_solution[0], a, j) * (va + vb_solution[0]) / 2 - pt) > 1e-10)
                    THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");

                if (std::abs(s_acc_time(va, vb_solution[1], a, j) * (va + vb_solution[1]) / 2 - pt) > 1e-10)
                    THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif


            }
            else{
                vb_solution_num = 1;
                if (l_va_to_vk > pt) {
                    vb_solution[0] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                    , vk, va);
                }
                else {
                    double B = -Z1;
                    double C = 2 * pt * a - va * Z1 - va * va;
                    vb_solution[0] = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                if (std::abs(s_acc_time(va, vb_solution[0], a, j) * (va + vb_solution[0]) / 2 - pt) > 1e-10)
                    THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif
            }
        }
        else {
            double T_va_to_vp1 = s_acc_time(va, vp1, a, j);
            double l_va_to_vp1 = T_va_to_vp1 * (va + vp1) / 2;

            if (l_va_to_vp1 <= pt) {
                vb_solution_num = 0;
            }
            else if (l_va_to_0 <= pt) {
                vb_solution_num = 2;
                vb_solution[0] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                , vk, va);

                if (l_va_to_vk <= pt) {
                    vb_solution[1] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                    , vk, vp1);
                }
                else {
                    double B = -Z1;
                    double C = 2 * pt * a - va * Z1 - va * va;
                    vb_solution[1] = (-B - safe_sqrt(B * B - 4 * C)) / 2;
                }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                if (std::abs(s_acc_time(va, vb_solution[0], a, j) * (va + vb_solution[0]) / 2 - pt) > 1e-10)
                    THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");

                if (std::abs(s_acc_time(va, vb_solution[1], a, j) * (va + vb_solution[1]) / 2 - pt) > 1e-10)
                    THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif
            }
            else {
                vb_solution_num = 1;
                vb_solution[0] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                , vp1, va);

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                if (std::abs(s_acc_time(va, vb_solution[0], a, j) * (va + vb_solution[0]) / 2 - pt) > 1e-10)
                    THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif
            }
        }


        TRange r[2];
        Size r_size = 0;

        if (vb_solution_num == 0) {
            r_size = 1;
            r[0] = TRange{ 0,std::numeric_limits<double>::infinity() };
        }
        else if (vb_solution_num == 1) {
            r_size = 1;
            r[0] = TRange{ 0,s_acc_time(va,vb_solution[0],a,j) };
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            if (vb_solution[0] > vb_max + 1e-10)
                THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif
        }
        else {
            if (vb_max > vb_solution[0]) {
                r_size = 2;
                r[0] = TRange{ 0,s_acc_time(va,vb_solution[0],a,j) };
                r[1] = TRange{ s_acc_time(va,vb_solution[1],a,j),std::numeric_limits<double>::infinity() };
            }
            else {
                r_size = 2;
                r[1] = TRange{ s_acc_time(va,vb_solution[1],a,j),std::numeric_limits<double>::infinity() };
            }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            if (vb_solution[1] > vb_max + 1e-10)
                THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif
        }




        std::vector<double> solutions;
        std::vector<int> slope_up; // l 随 T 的slope，为正时， T 越大 l 约长

        // STEP1 先求 vb 在 [va - a*a/j，va]中的解
        if (vk <= vp1) {
            // 可能有 2 个解，位于【va - a*a/j , va/3】与【va/3 , va】中
            if (l_va_to_vp1 >= pt) {
                // 一定有第一组解 //
                double vb = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                , vp1, va);

                solutions.push_back(vb);
                slope_up.push_back(1);

                // 可能有第二组解 //
                if ((vk >= 0 && l_va_to_vk <= pt) || (vk<0 && l_va_to_0 <= pt)) {
                    double vb = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                    , std::max(vk,0.0), va);

                    solutions.push_back(vb);
                    slope_up.push_back(-1);
                }
            }
        }
        else {
            // 可能有 1 个解，位于[va - a*a/j , va] 中
            if ((vk >= 0 && l_va_to_vk >= pt) || (vk < 0 && l_va_to_0 >= pt)) {
                double vb = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                , vk, va);

                solutions.push_back(vb);
                slope_up.push_back(1);
            }
        }

        // STEP2 求 vb 在 [0, va - a*a / j] 中的解
        if (vk >= vp2) {
            //  可能有 2 个解
            double B = -Z1;
            double C = 2 * pt * a - va * Z1 - va * va;
            if (l_va_to_vk < pt && l_va_to_vp2 >= pt) {
                double vb = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                solutions.push_back(vb);
                slope_up.push_back(1);
            }

            double vb = (-B - safe_sqrt(B * B - 4 * C)) / 2;
            solutions.push_back(vb);
            slope_up.push_back(-1);
        }
        else {
            // 只有一个解
            double B = -Z1;
            double C = 2 * pt * a - va * Z1 - va * va;
            double vb = (-B - safe_sqrt(B * B - 4 * C)) / 2;
            solutions.push_back(vb);
            slope_up.push_back(-1);
        }


        std::vector<double> real_solutions;
        std::vector<int> real_slope_up; // l 随 T 的slope，为正时， T 越大 l 约长

        for (int i = 0; i < solutions.size(); ++i) {
            if (solutions[i] <= vb_max || solutions[i] >= 0) {
                real_solutions.push_back(solutions[i]);
                real_slope_up.push_back(slope_up[i]);
            }
        }

        if (real_solutions.size() == 0) {
            ret.push_back(TRange{ 0, std::numeric_limits<double>::infinity() });
        }
        if (real_solutions.size() == 1) {
            ret.push_back(TRange{ 0, s_acc_time(va, real_solutions[0],a,j) });
        }
        if (real_solutions.size() == 2) {
            ret.push_back(TRange{ 0, s_acc_time(va, real_solutions[0],a,j) });
            ret.push_back(TRange{ s_acc_time(va, real_solutions[1],a,j), std::numeric_limits<double>::infinity() });

        }
        if (real_solutions.size() == 3) {
            ret.push_back(TRange{ 0, s_acc_time(va, real_solutions[0],a,j) });
            ret.push_back(TRange{ s_acc_time(va, real_solutions[1],a,j), s_acc_time(va, real_solutions[2],a,j) });
        }
        if (real_solutions.size() == 4) {
            ret.push_back(TRange{ 0, s_acc_time(va, real_solutions[0],a,j) });
            ret.push_back(TRange{ s_acc_time(va, real_solutions[1],a,j), s_acc_time(va, real_solutions[2],a,j) });
            ret.push_back(TRange{ s_acc_time(va, real_solutions[3],a,j), std::numeric_limits<double>::infinity() });
        }


        // CASE 1中的极大值
        double vb = std::max(va / 3, va - a*a / j);
        double l;
        if (vb <= vb_max) {
            l = s_acc_time(va, vb, a, j) * (va + vb) / 2;
            if (l > pt) {
                vb = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt;}
                , vb, va);
                return s_acc_time(va, vb, a, j);
            }
        }


        vb = std::min({ a*a / (2 * j), va - a*a / j, vb_max });
        if (vb >= 0 && vb <= vb_max) {
            l = s_acc_time(va, vb, a, j) * (va + vb) / 2;
            if (l > pt) {
                double B = -Z1;
                double C = 2 * pt * a - va * Z1 - va * va;
                vb = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                return s_acc_time(va, vb, a, j);
            }
        }

        return std::numeric_limits<double>::infinity();


        if (l_va_to_0 <= pt)
            return std::numeric_limits<double>::infinity();






        else {

            if (va<a * a / j || (2 * a * va) / j - a * a * a / j / j > pt) {
                //% 此时无匀速段
                //% 前进时间为：t = 2 * sqrt( (va-vb) / j );
                //% 前进长度的平方为：
                //% p^2 = (t * (va+vb)/2)^2
                //%     =  va^3/j + (va^2*vb)/j - (va*vb^2)/j - vb^3/j
                //% vb 需求解一元三次方程 【1,va,-va^2,pb-pa-va^3】
                //%
                //% vb 取尽可能大的实数
                //% vb 的范围取自 【va/3，va】,
                //% 因为T的极值为 sqrt(va*8/3/j)，此时带入vb的公式，可得

                double vb = newton_raphson_binary_search([va, j, pt](double x) {return safe_sqrt((va - x) / j) * (va + x) - pt; }
                , va / 3, va);

                //double vb2 = newton_raphson_binary_search([va, j, pt](double x) {return safe_sqrt((va - x) / j) * (va + x) - pt; }
                //, 0, va/3);
                return s_acc_time(va, vb, a, j);
            }
            else {
                //% 此时有匀速段
                //% 前进时间为：t = (va-vb)/a+a/j;
                //% 前进长度为：
                //% l = t*(va+vb)/2
                //%   = - vb^2/(2*a) + (a*vb)/(2*j) + (va*(a/j + va/a))/2
                //% vb 需求解一元二次方程 【
                //%       -1/(2*a),
                //%       a/(2*j),
                //%       (va*(a/j + va/a))/2-pt
                //% 】
                //% 对于根来说，应当取大值，这是因为Tmax应该尽可能的小
                double B = -Z1;
                double C = 2 * pt * a - va * Z1 - va * va;
                double vb = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                return s_acc_time(va, vb, a, j);
            }
        }
    }
    auto s_scurve_cpt_T_below(const SCurveParam& param) -> double {
        const double va = param.va_;
        const double vb_max = param.vb_max_;
        const double vc_max = param.vc_max_;
        const double a = param.a_;
        const double j = param.j_;
        const double pt = param.pb_ - param.pa_;

        double Tmin, vb, l, v1, v2, v_upper, v_below;

        double Z1 = a * a / j;
        double T_va_to_vb = s_acc_time(va, vb_max, a, j);
        double l_va_to_vb = T_va_to_vb * (va + vb_max) / 2;

        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 计算Tmin：%%%%%%

        //% ------------------ l1 --------------------
        //% 加速不到max_vb, 无法达到最大加速度a
        //% va < vb < max_vb, vb - va < a^2 / j
        vb = std::min(va + a * a / j, vb_max);
        l = std::min(s_acc_time(va, vb, a, j) * (va + vb) / 2, l_va_to_vb);
        if (va < vb_max && pt < l) {
            //%     %%%%%%%%%%%%%%%%% METHOD1 %%%%%%%%%%%%%%%%% 
            //%     % clear
            //%     % syms va vb a j pt
            //%     % T = 2 * sqrt((vb-va) / j )
            //%     % l = T * (va+vb)/2
            //%     % expand((T*(va + vb)/2)^2 *j - pt^2*j)
            //%     % coeffs(l^2*j - pt^2*j, vb)
            //%     %
            //%     % k3 = 1;
            //%     % k2 = va;
            //%     % k1 = -va^2;
            //%     % k0 = -j*pt^2 - va^3;
            //%     %
            //%     % f(vb) = k3*vb^3 + k2*vb^2 + k1*vb + k0 
            //%     % 
            //%     % 对f求导，可知
            //%     % df    = 3*k3*vb^2 + 2*k2*vb + k1
            //%     %
            //%     % 其极值为 -va 与 1/3va
            //%     % 将此2值带入，可以看出
            //%     % f(-va)  = -j*pt^2
            //%     % f(va/3) = - j*pt^2 - (32*va^3)/27
            //%     % 因此该三次方程在pt不为0时只有一个根，为0时有3个根，此时选择最大的
            //%     k3 = 1;
            //%     k2 = va;
            //%     k1 = -va^2;
            //%     k0 = -j*pt^2 - va^3;
            //%     
            //%     r = cubic_equation_solve(k3,k2,k1,k0);
            //%     % 选根 %
            //%     % 【注意】 pt为0时有3个根，两个为 -va，因此这里选择最大的
            //%     if(isreal(r(3)))
            //%         vb = r(3);
            //%     else
            //%         vb = r(1);
            //%     end

            //%%%%%%%%%%%%%%%%% METHOD2 %%%%%%%%%%%%%%%%% 
            //% newton raphson %
            vb = newton_raphson_binary_search([va, j, pt](double x) {return safe_sqrt((x - va) / j) * (va + x) - pt; }
            , va, vb_max);
            return s_acc_time(va, vb, a, j);
        }

        //% ------------------ l2 --------------------
        //% 加速不到max_vb, 可以达到最大加速度a
        //% va < vb < max_vb, vb - va >= a^2 / j
        l = l_va_to_vb;
        if (va < vb_max && pt < l) {
            //% clear
            //% syms va vb a j pt;
            //% T = (vb-va) / a + a/j;
            //% l = T * (va+vb)/2;
            //% coeffs(l,vb)
            //% 
            //% k2 = 1/(2*a);
            //% k1 = a/(2*j);
            //% k0 = (va*(a/j - va/a))/2 - pt;
            //% 
            //% 其极值为 vb = -k1/2/k2 = -a^2/(2*j)
            //% 因此取其右侧大值
            double k2 = 1.0 / (2 * a);
            double k1 = a / (2 * j);
            double k0 = (va * (a / j - va / a)) / 2 - pt;

            //% 选根 %
            vb = (-k1 + safe_sqrt(k1 * k1 - 4 * k2 * k0)) / (k2 * 2);
            Tmin = s_acc_time(va, vb, a, j);
            return Tmin;
        }

        //% ------------------ l3 --------------------
        //% 可以加速到某个v，无匀速段，之后减速到max_vb，加减速过程，加速度均不超过a
        //% v1 = max(va,vb), v2 = min(va,vb), v1 < v < min(v2 + a^2/j, max_v)
        vb = vb_max;
        v1 = std::max(va, vb);
        v2 = std::min(va, vb);
        v_upper = std::min(v2 + Z1, vc_max);
        v_below = v1;
        l = -1;
        if (v_upper >= v_below) {
            l = s_acc_time(v1, v_upper, a, j) * (v_upper + v1) / 2 +
                s_acc_time(v2, v_upper, a, j) * (v_upper + v2) / 2;
        }
        if (pt < l) {
            //% 以下为方程求解
            //% syms v1 v2 a j pt v;
            //% T1 = 2 * sqrt((v-v1) / j )
            //% T2 = 2 * sqrt((v-v2) / j )
            //% l = T1 * (v1 + v) / 2 + T2 * (v2 + v) / 2
            //% 
            //% 其导数为：
            //% diff(l,v) == T1/2 + T2/2 + (v + v1)/(j*T1) + (v + v2)/(j*T2)
            //% 
            //
            //  考虑到sqrt在0附近数值求解的稳定性，因此设置 x = v-v1，以防在 v 接近 v1 的情况下
            //  ，sqrt(x - v1)精度不够
            // 
            double v_minus_v1 = newton_raphson_binary_search([v1, v2, j, pt](double x) {
                return safe_sqrt(x / j) * (2.0 * v1 + x) + safe_sqrt((v1 - v2 + x) / j) * (v1 + v2 + x) - pt;
                }
            , 0.0, v_upper - v_below);

            double T1 = 2 * safe_sqrt(v_minus_v1 / j);
            double T2 = 2 * safe_sqrt((v1 - v2 + v_minus_v1) / j);
            Tmin = T1 + T2;
            return Tmin;
        }

        //% ------------------ l4 --------------------
        //% 可以加速到某个v，无匀速段，之后减速到max_vb，加减速过程，有一段可以达到加速度a
        //% v1 = max(va,vb), v2 = min(va,vb), v1 < v < min(v2 + a^2/j, max_v)
        v1 = std::max(va, vb);
        v2 = std::min(va, vb);
        v_upper = std::min(v1 + Z1, vc_max);
        v_below = std::max(v2 + Z1, v1);
        l = -1;
        if (v_upper >= v_below) {
            l = s_acc_time(v1, v_upper, a, j) * (v_upper + v1) / 2 +
                s_acc_time(v2, v_upper, a, j) * (v_upper + v2) / 2;
        }
        if (pt < l) {
            //% 以下为方程求解
            //% syms v1 v2 a j pt v;
            //% T1 = 2 * sqrt((v-v1) / j )
            //% T2 = (v-v2) / a + a/j
            //% l1 = T1 * (v1 + v) / 2
            //% l2 = T2 * (v2 + v) / 2
            //% l  = l1 + l2
            //% 
            //% 可以化成1元4次方程：
            //% eq = expand(l1^2 - (pt-l2)^2)
            //% 
            //% 受限于难以求解1元4次方程，因此还是使用牛顿法 
            //%
            //% eq = l - pt
            //% >> eq  = sqrt((x-v1)/j)*(v1+x) + ((x-v2)/a+a/j)*(v2+x)/2 - pt)
            //% deq = diff(eq,v)
            //% >> deq = T1/2 + T2/2 + (v + v2)/(2*a) + (v+v1)/(j*T1)
            //% 
            //  考虑到sqrt在0附近数值求解的稳定性，因此设置 x = v-v1，以防在 v 接近 v1 的情况下
            //  ，sqrt(x - v1)精度不够
            // 
            double v_minus_v1 = newton_raphson_binary_search([v1, v2, a, j, pt](double x)->double {
                return safe_sqrt(x / j) * (x + 2.0 * v1) + ((v1 - v2 + x) / a + a / j) * (v1 + v2 + x) / 2 - pt;
                }
            , v_below - v1, v_upper - v1);

            double T1 = 2 * safe_sqrt(v_minus_v1 / j);
            double T2 = (v1 - v2 + v_minus_v1) / a + a / j;
            Tmin = T1 + T2;
            return Tmin;
        }

        //% ------------------ l5 --------------------
        //% 可以加速到某个v，无匀速段，之后减速到max_vb，加减速过程，两段都可以达到加速度a
        //% v1 = max(va,vb), v2 = min(va,vb), v1 + a^2/j < v < max_v
        v1 = std::max(va, vb);
        v2 = std::min(va, vb);
        v_upper = vc_max;
        v_below = v1 + Z1;
        l = -1;
        if (v_upper >= v_below) {
            l = s_acc_time(v1, v_upper, a, j) * (v_upper + v1) / 2 +
                s_acc_time(v2, v_upper, a, j) * (v_upper + v2) / 2;
        }
        if (pt < l) {
            //% 以下为方程求解
            //% syms v1 v2 a j pt v;
            //% T1 = (v-v1) / a + a/j
            //% T2 = (v-v2) / a + a/j
            //% l1 = T1 * (v1 + v) / 2
            //% l2 = T2 * (v2 + v) / 2
            //% l  = l1 + l2
            //% 
            //% 可以化成1元2次方程：
            //% f(v) = l-pt
            //% f(v) == 0
            //% 
            //% 其系数为：
            //% k2 = 1/a
            //% k1 = a/j
            //% k0 = (v1*(a/j - v1/a))/2 - pt + (v2*(a/j - v2/a))/2
            //%
            //% 分析 f 的极值，有：
            //% df = diff(f,v)
            //% solve(df,v)
            //%
            //% ans = -a^2/(2*j)
            //%
            //% 因此f的根位于上述结果两侧，应取较大值
            double k2 = 1 / a;
            double k1 = a / j;
            double k0 = (v1 * (a / j - v1 / a)) / 2 - pt + (v2 * (a / j - v2 / a)) / 2;

            double v = (-k1 + safe_sqrt(k1 * k1 - 4 * k0 * k2)) / (2 * k2);

            double T1 = (v - v1) / a + a / j;
            double T2 = (v - v2) / a + a / j;
            Tmin = T1 + T2;
            return Tmin;
        }

        //% ------------------ l6 --------------------
        //% 可以加速到max_v，匀速运行一段时间，之后减速到max_vb
        //% v1 = max(va,vb), v2 = min(va,vb), v = max_v
        v1 = std::max(va, vb);
        v2 = std::min(va, vb);
        double v = vc_max;
        double T1 = s_acc_time(v1, vc_max, a, j);
        double T2 = s_acc_time(v2, vc_max, a, j);
        double T3 = (pt - T1 * (v + v1) / 2 - T2 * (v + v2) / 2) / vc_max;
        Tmin = T1 + T2 + T3;
        return Tmin;
    }
    auto s_scurve_cpt_T_range(SCurveParam& param, double lcons = 1e-10, double vcons = 1e-7, double tcons = 1e-7) -> TSet {
        
        const double va = param.va_;
        const double vb_max = param.vb_max_;
        const double vc_max = param.vc_max_;
        const double a = param.a_;
        const double j = param.j_;
        const double pt = param.pb_ - param.pa_;

        //% 计算一定可行的 va_upper
        double va_upper;
        if (pt < (a * (a * a / j + 2 * vb_max)) / j) {
            //% clear
            //% syms T va pt j T a
            //% vb = va + j * T * T / 4;
            //% l = T * (vb + va) / 2;
            //% collect(l, T)
            //% solve(l == pt, T)
            double T = newton_raphson_binary_search([j, vb_max, pt](double T) { return j * T * T * T + 8 * vb_max * T - 8 * pt; }, 0, 2 * a / j);
            va_upper = std::min(param.va_upper_, vb_max + j * T * T / 4);
        }
        else {
            //% clear
            //% syms T va pt j T a
            //% vb = va + T * a - a ^ 2 / j;
            //% l = T * (vb + va) / 2;
            //% collect(l, T)
            //% solve(l == pt, T)
            double T = (a*a - 2 * j * vb_max + safe_sqrt((a*a - 2*j*vb_max)*(a*a - 2*j*vb_max) + 8*pt*a*j*j)) / (2 * a * j);
            va_upper = std::min(param.va_upper_, vb_max + T * a - a*a / j);
        }

        // 失败
        if (va_upper < param.va_below_ - vcons) {
            return TSet();
        }

        TSet t_set;
        t_set.reserve(2);

        // T_below 惟一
        param.va_ = va_upper;
        double T_below = s_scurve_cpt_T_below(param);

        // T_upper 不一定
        param.va_ = param.va_below_;
        {
            const double va = param.va_;
            const double vb_max = param.vb_max_;
            const double a = param.a_;
            const double j = param.j_;
            const double pt = param.pb_ - param.pa_;

            const double Z1 = a * a / j;
            
            //% 计算vb
            //  因为可能有多解需要计算出距离 vb_max 最近的 vb 值
            // 
            // 1）在 vb >= va - a^2/j 时，此时没有匀加速段
            //% 此时无匀加速段
            //% 前进时间为：t = 2 * sqrt( (va-vb) / j );
            //% 前进长度的平方为：
            //% p^2 = (t * (va+vb)/2)^2
            //%     =  va^3/j + (va^2*vb)/j - (va*vb^2)/j - vb^3/j
            //% vb 需求解一元三次方程 【1,va,-va^2,pb-pa-va^3】
            //%
            //% vb 可能有两个解，分别位于 【0,va/3】与【va/3，va】中
            // 
            // 1.1）va/3 <= va - a^2/j
            //   l 随 vb 在[va/3，va]单调递减，极大值位于 vb = va/3 处，极小值位于 vb = va处 
            //   考虑 vb 应当尽量贴近 vb_max，因此不考虑 vb < va/3 时的根
            // 
            // 1.2）va/3 > va - a^2/j
            //   l 随 vb 在[va - a^2/j，va]单调递减，极大值位于 vb = va - a^2/j 处，极小值位于 vb = va处
            // 
            // 2）在 vb <  va - a^2/j 时，此时拥有匀加速段
            //% 此时有匀速段
            //% 前进时间为：t = (va-vb)/a+a/j;
            //% 前进长度为：
            //% l = t*(va+vb)/2
            //%   = - vb^2/(2*a) + (a*vb)/(2*j) + (va*(a/j + va/a))/2
            //% vb 需求解一元二次方程 【
            //%       -1/(2*a),
            //%       a/(2*j),
            //%       (va*(a/j + va/a))/2-pt
            //% 】
            //% 对于根来说，应当取大值，这是因为Tmax应该尽可能的小
            // 其极值应当位于 a^2/(2*j) 处
            // 
            // 2.1）a^2/(2*j) <= va - a^2/j
            //   l 随 vb 在[a^2/(2*j)，va - a^2/j]单调递减，极大值位于 vb = a^2/(2*j) 处，极小值位于 vb = va - a^2/j 处 
            //   考虑 vb 应当尽量贴近 vb_max，因此不考虑 vb < a^2/(2*j) 时的根
            // 
            // 2.2）a^2/(2*j) >  va - a^2/j
            //   l 随 vb 在[0, va - a^2/j]单调递增，极大值位于 vb = va - a^2/j 处，极小值位于 vb = 0处
            // 
            //% 【条件1】 加速度正好可以达到a时，所前进的长度
            //% 此时 vb = - a^2/j + va
            //% 前进时间 t = (va-vb)/a+a/j
            //% 前进长度 l = t*(va+vb)/2 = (2*a*va)/j - a*a*a/j/j


            double T_va_to_0 = s_acc_time(va, 0, a, j);
            double l_va_to_0 = T_va_to_0 * va / 2;

            double T_va_to_vbmax = s_acc_time(va, vb_max, a, j);
            double l_va_to_vbmax = T_va_to_vbmax * (va + vb_max) / 2;

            double vk = va - a * a / j;
            double T_va_to_vk = s_acc_time(va, vk, a, j);
            double l_va_to_vk = T_va_to_vk * (va + vk) / 2;

            // v_peak at case 1
            double vp1 = va / 3;
            double T_va_to_vp1 = s_acc_time(va, vp1, a, j);
            double l_va_to_vp1 = T_va_to_vp1 * (va + vp1) / 2;

            // v_peak at case 2
            double vp2 = a * a / (2 * j);
            double T_va_to_vp2 = s_acc_time(va, vp2, a, j);
            double l_va_to_vp2 = T_va_to_vp2 * (va + vp2) / 2;

            // vb 的三个特殊值
            // vk : va - a*a/j, 为是否有匀减速的分界点
            // vp1: va / 3, 没有匀减速时，l取极值时的 vb 值
            // vp2: a*a/j/2 有匀减速时，l取极值时的vb
            //
            // 判断vk 与 vp1 vp2的关系
            // vk = 3*vp1 - 2*vp2
            // 
            // vk - vp1 = 2*（vp1-vp2）
            // vk - vp2 = 3*（vp1-vp2）
            //
            // 若   vp1 >= vp2
            // 则   vk >= vp1 >= vp2
            // 否则 vk < vp1 < vp2
            //
            // 因此至多有2个根
            //
            // CASE 1 vp1 >= vp2:
            //    CASE 1.0     l_vp1 <= pt
            //          T取值为：[Tmin，inf]
            //    CASE 1.1     l_vp1 > pt && pt <= l_0
            //          T取值为：[Tmin，s1] 并 [s2，inf]
            //    CASE 1.2     pt < l_0
            //          T取值为：[Tmin，s1]
            // CASE 2 vp < vp1:
            //    CASE 1.0     l_vp2 <= pt
            //          T取值为：[Tmin，inf]
            //    CASE 1.1     l_vp2 > pt && pt <= l_0
            //          T取值为：[Tmin，s1] 并 [s2，inf]
            //    CASE 1.2     pt < l_0
            //          T取值为：[Tmin，s1]

            double vb_solution[2];
            double vb_solution_num;
            if (vp1 >= vp2) {
                // 此时 vk >= vp1 >= vp2，可以达到 vp2
                double T_va_to_vp2 = s_acc_time(va, vp2, a, j);
                double l_va_to_vp2 = T_va_to_vp2 * (va + vp2) / 2;

                if (l_va_to_vp2 <= pt + lcons) {
                    vb_solution_num = 0;
                }
                else if (l_va_to_0 <= pt + lcons) {
                    vb_solution_num = 2;
                    if (l_va_to_vk > pt) {
                        vb_solution[0] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                        , vk, va);
                    }
                    else {
                        double B = -Z1;
                        double C = 2 * pt * a - va * Z1 - va * va;
                        vb_solution[0] = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                    }

                    double B = -Z1;
                    double C = 2 * pt * a - va * Z1 - va * va;
                    vb_solution[1] = (-B - safe_sqrt(B * B - 4 * C)) / 2;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                    if (std::abs(s_acc_time(va, vb_solution[0], a, j) * (va + vb_solution[0]) / 2 - pt) > 1e-10)
                        THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");

                    if (std::abs(s_acc_time(va, vb_solution[1], a, j) * (va + vb_solution[1]) / 2 - pt) > 1e-10)
                        THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif


                }
                else {
                    vb_solution_num = 1;
                    if (l_va_to_vk > pt) {
                        vb_solution[0] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                        , vk, va);
                    }
                    else {
                        double B = -Z1;
                        double C = 2 * pt * a - va * Z1 - va * va;
                        vb_solution[0] = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                    }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                    if (std::abs(s_acc_time(va, vb_solution[0], a, j) * (va + vb_solution[0]) / 2 - pt) > 1e-10)
                        THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif
                }
            }
            else {
                double T_va_to_vp1 = s_acc_time(va, vp1, a, j);
                double l_va_to_vp1 = T_va_to_vp1 * (va + vp1) / 2;

                if (l_va_to_vp1 <= pt + lcons) {
                    vb_solution_num = 0;
                }
                else if (l_va_to_0 <= pt + lcons) {
                    vb_solution_num = 2;
                    vb_solution[0] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                    , vp1, va);

                    if (l_va_to_vk <= pt) {
                        vb_solution[1] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                        , std::max(vk, 0.0), vp1);
                    }
                    else {
                        double B = -Z1;
                        double C = 2 * pt * a - va * Z1 - va * va;
                        vb_solution[1] = (-B - safe_sqrt(B * B - 4 * C)) / 2;
                    }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                    double l = s_acc_time(va, vb_solution[0], a, j) * (va + vb_solution[0]) / 2;
                    if (std::abs(l - pt) > 1e-10) {
                        s_scurve_cpt_T_range( param);
                        THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
                    }
                        
                    l = s_acc_time(va, vb_solution[1], a, j) * (va + vb_solution[1]) / 2;
                    if (std::abs(s_acc_time(va, vb_solution[1], a, j) * (va + vb_solution[1]) / 2 - pt) > 1e-10)
                        THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif
                }
                else {
                    vb_solution_num = 1;
                    vb_solution[0] = newton_raphson_binary_search([va, j, pt](double x)->double { return safe_sqrt((va - x) / j) * (va + x) - pt; }
                    , vp1, va);

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                    if (std::abs(s_acc_time(va, vb_solution[0], a, j) * (va + vb_solution[0]) / 2 - pt) > 1e-10)
                        THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
#endif
                }
            }


            if (vb_solution_num == 0) {
                t_set.push_back(TRange{ T_below,std::numeric_limits<double>::infinity() });
            }
            else if (vb_solution_num == 1) {
                if (vb_solution[0] > vb_max + vcons)
                    t_set.clear();
                else
                    t_set.push_back(TRange{ T_below,s_acc_time(va,std::min(vb_solution[0], vb_max),a,j) });
                    
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                if (vb_solution[0] > vb_max + 1e-7 && vb_max > 1e-7) {
                    s_scurve_cpt_T_range(param);
                    THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
                }
#endif
            }
            else {
                if (vb_solution[0] < vb_max + vcons) {
                    t_set.push_back(TRange{ T_below,s_acc_time(va,vb_solution[0],a,j) });
                    t_set.push_back(TRange{ s_acc_time(va,vb_solution[1],a,j),std::numeric_limits<double>::infinity() });
                }
                else if(vb_solution[1] < vb_max + vcons){
                    t_set.push_back(TRange{ s_acc_time(va,vb_solution[1],a,j),std::numeric_limits<double>::infinity() });
                }
                else {
                    t_set.push_back(TRange{ s_acc_time(va,vb_max,a,j),std::numeric_limits<double>::infinity() });
                }

//#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
//                if (t_set.empty())
//                    THROW_FILE_LINE("ERROR s_scurve_cpt_T_upper");
//#endif
            }
        }


        return t_set;
    }

    // 必然成功
    auto s_scurve_cpt_vavc(SCurveParam& param) -> void {
        //% 在给定过程中起始速度va, 最大速度v，加速度a，跃度j，时间长度T的情况下
        //% 自动计算末端所可能达到的【最大的】vb
        //%
        //% t      : current time
        //% pa : init pos
        //% va : init vel
        //% pb : end  pos
        //% v : max vel  during period
        //% a : max acc  during period
        //% j : max jerk during period
        //% T : period

        const double pa = param.pa_;
        const double pb = param.pb_;
        const double pt = param.pb_ - param.pa_;
        const double vb = param.vb_;
        const double a = param.a_;
        const double j = param.j_;
        const double T = param.T_;
        const double vc_max = param.vc_max_;
        constexpr double cons = 10000 * std::numeric_limits<double>::epsilon();

        //% 根据速度可达修正 va_range
        double va_upper = std::min(param.va_upper_, s_acc_vend(param.vb_, a, j, T));
        double va_below = std::max(param.va_below_, s_acc_vend(param.vb_, -a, -j, T));
        auto& va = param.va_;
        auto& vc = param.vc_;
        auto& Ta = param.Ta_;
        auto& Tb = param.Tb_;
        auto& mode = param.mode_;

        //% CASE 1: vc > max(vb, va_range)
        //% va = va_upper
        //% vc > max(va_upper, vb)
        double T_va_upper_to_vb = s_acc_time(va_upper, vb, a, j);
        double l_upper = std::max(va_upper, vb) * (T - T_va_upper_to_vb) + T_va_upper_to_vb * (vb + va_upper) / 2;
        if (pt > l_upper) {
            //% 如果用 vc 做未知数，用 newton - ranphson 方法，在vc 很接近v2的时候数值性能很差
               //% 例如：
            va = va_upper;
            double vc_upper;
            s_scurve_cpt_vc_upper_by_va_vb_T(va, vb, T, a, j, &vc_upper);

            vc = newton_raphson_binary_search([va, vb, a, j, pt, T](double vc) {
                return s_acc_time(va, vc, a, j) * (va + vc) / 2
                    + s_acc_time(vb, vc, a, j) * (vb + vc) / 2
                    + std::max(T - s_acc_time(va, vc, a, j) - s_acc_time(vb, vc, a, j), 0.0) * vc
                    - pt;
                }
            , std::max(va, vb), std::min(vc_max, vc_upper));
            Ta = s_acc_time(va, vc, a, j);
            Tb = s_acc_time(vb, vc, a, j);
            mode = 0;

            //% va = va_upper;
            //% v1 = min(va, vb);
            //% v2 = max(va, vb);
            //%
            //%% 事实上 newton - ranphson 内的方程应该为：
            //% %vc = s_acc_vend(v2, a, j, T2);
            //%% T1 = s_acc_time(v1, vc, a, j);
            //%%
            //%% T1* (v1 + vc) / 2 + T2 * (v2 + vc) / 2 + (T - T2 - T1) * vc - pt
            //%
            //%%但matlab似乎只支持单行函数
            //% T2_upper = min(newton_raphson_binary_search(@(T2)(...
            //%         s_acc_vend(v2, a, j, T2) - s_acc_vend(v1, a, j, T - T2)) ...
            //%, 0, T...
            //%, eps), s_acc_time(v2, vc_max, a, j));
            //%
            //% T2 = newton_raphson_binary_search(@(T2)(...
            //%         s_acc_time(v1, s_acc_vend(v2, a, j, T2), a, j) * (v1 + s_acc_vend(v2, a, j, T2)) / 2  ...
            //%         +T2 * (v2 + s_acc_vend(v2, a, j, T2)) / 2  ...
            //%         +(T - T2 - s_acc_time(v1, s_acc_vend(v2, a, j, T2), a, j)) * s_acc_vend(v2, a, j, T2) ...  % 本行不同
            //% -pt) ...
            //%, 0, T2_upper...
            //%, 10 * eps);
            //%
            //%
            //% vc = s_acc_vend(v2, a, j, T2);
            //% T1 = s_acc_time(v1, vc, a, j);
            //% if (va < vb)
            //% Ta = T1;
            //% Tb = T2;
            //% else
            //% Tb = T1;
            //% Ta = T2;
            //% end

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% debug check%
            double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0 + (T - Ta - Tb) * vc;
            //% if (vc < 0 || vc > vc_max || abs(l - pt) > cons || (T - Ta - Tb) < -cons)
            if (vc < -cons || vc > vc_max + cons || std::abs(l - pt) > std::max(pt, 1.0) * 1e-10) {
                THROW_FILE_LINE("wrong in s_scurve_cpt_vavc CASE 1");
            }
#endif

            return;
        }
           

        //% CASE 2: vc < min(vb, va_range)
        //% va = va_below
        //% vc < min(va_below, vb)
        double T_va_below_to_vb = s_acc_time(va_below, vb, a, j);
        double l_below = std::min(va_below, vb) * (T - T_va_below_to_vb) + T_va_below_to_vb * (vb + va_below) / 2;
        if (pt < l_below) {
            //% 如果用 vc 做未知数，用 newton - ranphson 方法，在vc 很接近v2的时候数值性能很差
            //% 例如：
            va = va_below;
            double vc_below;
            s_scurve_cpt_vc_below_by_va_vb_T(va, vb, T, a, j, &vc_below);

            vc = newton_raphson_binary_search([va, vb, a, j, pt, T](double vc){
                return s_acc_time(va, vc, a, j)* (va + vc) / 2  
                    + s_acc_time(vb, vc, a, j) * (vb + vc) / 2  
                    + (T - s_acc_time(va, vc, a, j) - s_acc_time(vb, vc, a, j)) * vc 
                    - pt;
                }
                , std::max(vc_below, 0.0), std::min(va, vb));
            Ta = s_acc_time(va, vc, a, j);
            Tb = s_acc_time(vb, vc, a, j);
            mode = 0;

            //% va = va_below;
            //% v1 = min(va, vb);
            //% v2 = max(va, vb);
            //%
            //%% 事实上 newton - ranphson 内的方程应该为：
            //% %vc = s_acc_vend(v1, -a, -j, T1);
            //%% T2 = s_acc_time(v2, vc, a, j);
            //%%
            //%% T1* (v1 + vc) / 2 + T2 * (v2 + vc) / 2 + (T - T2 - T1) * vc - pt
            //%
            //%%但matlab似乎只支持单行函数
            //% T1_upper = min(newton_raphson_binary_search(@(T1)(...
            //%         s_acc_vend(v1, -a, -j, T1) - s_acc_vend(v2, -a, -j, T - T1)) ...
            //%, 0, T...
            //%, eps), s_acc_time(v1, 0, a, j));
            //%
            //% T1 = newton_raphson_binary_search(@(T1)(...
            //%         T1 * (v1 + s_acc_vend(v1, -a, -j, T1)) / 2  ...
            //%         +s_acc_time(v2, s_acc_vend(v1, -a, -j, T1), a, j) * (v2 + s_acc_vend(v1, -a, -j, T1)) / 2  ...
            //%         +(T - T1 - s_acc_time(v2, s_acc_vend(v1, -a, -j, T1), a, j)) * s_acc_vend(v1, -a, -j, T1) ...  % 本行不同
            //% -pt) ...
            //%, 0, T1_upper...
            //%, 10 * eps);
            //%
            //% vc = s_acc_vend(v1, -a, -j, T1);
            //% T2 = s_acc_time(v2, vc, a, j);
            //% if (va < vb)
            //% Ta = T1;  
            //% Tb = T2;
            //% else
            //% Tb = T1;
            //% Ta = T2;
            //% end

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% debug check%
            double l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0 + (T - Ta - Tb) * vc;
            if (vc < 0 || vc > vc_max + cons || std::abs(l - pt) > 1e-10 * std::max(pt, 1.0)) {
                THROW_FILE_LINE("wrong in s_scurve_cpt_vavc CASE 2");
            }
                
#endif
            return;
        }
            




        //% CASE 3     : min(vb, va_range) < vc < max(vb, va_range)
        //% 3.1 : vb < va_below
        //% 3.1.1 : vb < vc < va_below
        //% 3.1.2 : va_below < vc < va_upper
        //%
        //%3.2 : va_upper < vb
        //% 3.1.1 : va_upper < vc < vb
        //% 3.1.2 : va_below < vc < va_upper
        //%
        //%3.3 : va_below < vb < va_upper

        //% CASE 3.1    
        if (vb <= va_below) {
            double l_mid = T_va_below_to_vb * (va_below + vb) / 2 + (T - T_va_below_to_vb) * va_below;
            if (pt < l_mid) {
                //% 3.1.1
                va = va_below;
                if (T - T_va_below_to_vb > 1e-10) {
                    vc = (pt - T_va_below_to_vb * (va + vb) / 2) / (T - T_va_below_to_vb);
                    Ta = std::abs((vc - vb) / (va - vb)) * (T - T_va_below_to_vb);
                    Tb = std::abs((vc - va) / (va - vb)) * (T - T_va_below_to_vb);
                }
                else {
                    vc = (vb + va_below) / 2;
                    Ta = (T - T_va_below_to_vb) / 2;
                    Tb = (T - T_va_below_to_vb) / 2;
                }
                mode = 1;
                return;
            }

            else {
                //% 3.1.2
                //% va * (T - Tb) + (va + vb) / 2 * Tb == pt
                //%
                va = newton_raphson_binary_search([vb, a, j, pt, T](double va) {
                    return s_acc_time(vb, va, a, j)* (va + vb) / 2  
                        + (T - s_acc_time(va, vb, a, j)) * va 
                        - pt;
                    }
                    , va_below, va_upper);
                vc = va;
                Ta = T - s_acc_time(vb, va, a, j);
                Tb = 0;
                mode = 1;
                return;
            }

        }
        else if (vb >= va_upper) {
            double l_mid = T_va_upper_to_vb * (va_upper + vb) / 2 + (T - T_va_upper_to_vb) * va_upper;
            if (pt > l_mid) {
                //% 3.2.1
                va = va_upper;
                if (T - T_va_upper_to_vb > 1e-10) {
                    vc = (pt - T_va_upper_to_vb * (va_upper + vb) / 2) / (T - T_va_upper_to_vb);
                    Ta = std::abs((vc - vb) / (va - vb)) * (T - T_va_upper_to_vb);
                    Tb = std::abs((vc - va) / (va - vb)) * (T - T_va_upper_to_vb);
                }
                else {
                    vc = (vb + va_upper) / 2;
                    Ta = (T - T_va_upper_to_vb) / 2;
                    Tb = (T - T_va_upper_to_vb) / 2;
                }
                mode = 1;
                return;
            }
                
            else {
                //% 3.2.2
                //% va * (T - Tb) + (va + vb) / 2 * Tb == pt
                //%
                va = newton_raphson_binary_search([vb, a, j, pt, T](double va) {
                    return s_acc_time(vb, va, a, j)* (va + vb) / 2  
                        + (T - s_acc_time(va, vb, a, j)) * va 
                        - pt;
                    }
                    , va_below, va_upper);
                vc = va;
                Ta = T - s_acc_time(vb, va, a, j);
                Tb = 0;
                mode = 1;
                return;
            }

        }
        else {
            va = newton_raphson_binary_search([vb, a, j, pt, T](double va) {
                return s_acc_time(vb, va, a, j)* (va + vb) / 2
                    + (T - s_acc_time(va, vb, a, j)) * va - pt;
                },
                va_below, va_upper);
            vc = va;
            Ta = T - s_acc_time(vb, va, a, j);
            Tb = 0;
            mode = 1;
            return;
        }
    }

    auto s_scurve_test_following_nodes(std::list<SCurveNode>::iterator begin, std::list<SCurveNode>::iterator end, double T_min_set) -> bool {
        // 达到最后一个节点 //
        if (begin == std::prev(end))
            return true;

        std::vector<double> T_uppers(begin->params_.size()), T_belows(begin->params_.size());
        TSet t_set{ TRange{0.0, std::numeric_limits<double>::infinity()} };
        for (aris::Size i = 0; i < begin->params_.size(); ++i) {
            auto next_iter = std::next(begin);
            
            // 计算 vb range
            s_scurve_cpt_vb_range(begin->params_[i]);

            // 更新 vb
            std::next(begin)->params_[i].va_ = begin->params_[i].vb_;
            std::next(begin)->params_[i].va_upper_ = begin->params_[i].vb_upper_;
            std::next(begin)->params_[i].va_below_ = begin->params_[i].vb_below_;

            // 计算 Tmax Tmin
            TSet t_set_ins = s_scurve_cpt_T_range(std::next(begin)->params_[i], 1e-11, 1e-8, 1e-8);
            t_set = s_scurve_cpt_intersection(t_set, t_set_ins, 1e-10);
        }


        if (t_set.size() == 0) {
            return false;
        }
        else {
            double Tmin_all = t_set.back().below_;
            double Tmax_all = t_set.back().upper_;

            Tmin_all = std::max(Tmin_all, T_min_set);

            if (Tmax_all == std::numeric_limits<double>::infinity()) {
                return true;
            }
            else if (Tmax_all < 0 || Tmin_all < 0 || Tmax_all < Tmin_all) {
                return false;
            }
            else {
                for (auto& param : std::next(begin)->params_) {
                    param.T_ = Tmax_all;
                }
                return s_scurve_test_following_nodes(std::next(begin), end, T_min_set);
            }
        }
    }

    // 循环计算每个节点2：
    auto ARIS_API s_scurve_make_nodes(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter, double T_min)->int {
        // 设置正确的 pa, 并检查 vc, a, j 等参数的合理性
        for (auto iter = std::next(begin_iter); iter != end_iter; ++iter) {
            // 设置正确的 pa
            for (Size i = 0; i < iter->params_.size(); ++i) {
                iter->params_[i].pa_ = std::prev(iter)->params_[i].pb_;

                if (iter->params_[i].pa_ > iter->params_[i].pb_)
                    THROW_FILE_LINE("POS SET NOT CORRECT");

                if (iter->params_[i].pb_ - iter->params_[i].pa_ >= 100 * std::numeric_limits<double>::epsilon()) {
                    if (iter->params_[i].vc_max_ < 1e-10)
                        THROW_FILE_LINE("VEL SET NOT CORRECT");
                    if (iter->params_[i].a_ < 1e-10)
                        THROW_FILE_LINE("ACC SET NOT CORRECT");
                    if (iter->params_[i].j_ < 1e-10)
                        THROW_FILE_LINE("JERK SET NOT CORRECT");
                }
            }
        }

        // 设置正确的 vb_max,  vb_max 应小于上一个与这一个的 较小的 vc //
        for (auto iter = begin_iter; iter != end_iter && iter != std::prev(end_iter); ++iter) {
            // 设置正确的 vb_max
            for (Size i = 0; i < iter->params_.size(); ++i) {
                iter->params_[i].vb_max_ = std::min(iter->params_[i].vc_max_, iter->params_[i].vb_max_);
                iter->params_[i].vb_max_ = std::min(std::next(iter)->params_[i].vc_max_, iter->params_[i].vb_max_);
            }
        }

        // 设置起始的 va_range 等于 va
        for (Size i = 0; i < begin_iter->params_.size(); ++i) {
            begin_iter->params_[i].va_upper_ = begin_iter->params_[i].va_;
            begin_iter->params_[i].va_below_ = begin_iter->params_[i].va_;
        }

        // 设置最后的 vb_max 等于 0
        for (Size i = 0; i < std::prev(end_iter)->params_.size(); ++i) {
            std::prev(end_iter)->params_[i].vb_max_ = 0.0;
            std::prev(end_iter)->params_[i].vb_upper_ = 0.0;
            std::prev(end_iter)->params_[i].vb_below_ = 0.0;
        }

        // 开始正向循环计算，更新每个节点的 T vb_range 等信息
        for (auto iter = begin_iter; iter != end_iter; ++iter) {
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            static int count_ = 0;
            std::cout << "trajectory count:" << count_ << std::endl;
            if (count_ >= 981)
                std::cout << "debuging" << std::endl;
            count_++;

#endif
            // STEP 1 : 计算全部末端的最大最小时间
            std::vector<double> Tmaxs(iter->params_.size()), Tmins(iter->params_.size());
            TSet t_set{ TRange{0.0, std::numeric_limits<double>::infinity()} };
            for (Size i = 0; i < iter->params_.size(); ++i) {
                // 设置正确的 pa 和 va
                if (iter != begin_iter) {
                    iter->params_[i].va_ = std::prev(iter)->params_[i].vb_;
                    iter->params_[i].va_upper_ = std::prev(iter)->params_[i].vb_upper_;
                    iter->params_[i].va_below_ = std::prev(iter)->params_[i].vb_below_;
                }

                // 计算 Tmax Tmin
                auto t_set_ins = s_scurve_cpt_T_range(iter->params_[i], 1e-10, 1e-7, 1e-7);
                t_set = s_scurve_cpt_intersection(t_set, t_set_ins, 1e-7);
            }


            if (t_set.size() == 0) {
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                TSet t_set{ TRange{0.0, std::numeric_limits<double>::infinity()} };
                for (Size i = 0; i < iter->params_.size(); ++i) {
                    // 设置正确的 pa 和 va
                    if (iter != begin_iter) {
                        iter->params_[i].va_ = std::prev(iter)->params_[i].vb_;
                        iter->params_[i].va_upper_ = std::prev(iter)->params_[i].vb_upper_;
                        iter->params_[i].va_below_ = std::prev(iter)->params_[i].vb_below_;
                    }

                    // 计算 Tmax Tmin
                    auto t_set_ins = s_scurve_cpt_T_range(iter->params_[i]);
                    t_set = s_scurve_cpt_intersection(t_set, t_set_ins);
                }
                std::cout << "error of t_Set" << std::endl;
                THROW_FILE_LINE("error of t_Set");
#endif
                return -1;
            }

            for (auto &range : t_set) {
                if (range.upper_ < T_min)
                    continue;
                
                double Tmin_all = std::max(range.below_, T_min);
                double Tmax_all = range.upper_;

                // STEP 2 : check 本区间是否可行，如果可行，将其上限设为有限值
                double T_upper = Tmax_all;
                double T_below = std::max(Tmin_all, T_min);
                if (T_upper == std::numeric_limits<double>::infinity()) {
                    // 本区间一定可行
                    T_upper = std::max(T_below, 1.0) * 2.0; // in case T_below == 0.0
                    for (auto& p : iter->params_)
                        p.T_ = T_upper;

                    while (!s_scurve_test_following_nodes(iter, end_iter, T_min)) {
                        T_below = T_upper;
                        T_upper = 2.0 * T_upper;

                        for (auto& p : iter->params_)
                            p.T_ = T_upper;
                    }
                }
                else {
                    // STEP 2.1 : 计算本区间是否可行
                    for (auto& p : iter->params_)
                        p.T_ = Tmax_all;
                    if (!s_scurve_test_following_nodes(iter, end_iter, T_min))
                        continue;
                }

                // STEP 3 : 二分法求解最优的可行时间
                double diff = std::abs(T_upper - T_below);
                double diff_last = 10 * diff;

                while (diff < diff_last) {
                    diff_last = diff;
                    double T_next = (T_upper + T_below) / 2;

                    for (auto& p : iter->params_)
                        p.T_ = T_next;

                    if (s_scurve_test_following_nodes(iter, end_iter, T_min)) {
                        T_upper = T_next;
                    }
                    else {
                        T_below = T_next;
                    }

                    diff = std::abs(T_upper - T_below);
                }

                // STEP 4 : 设置可行时间并计算每段 s 曲线
                for (auto& p : iter->params_) {
                    p.T_ = T_upper;
                    s_scurve_cpt_vb_range(p);

                    if (iter != begin_iter) {
                        LargeNum t0 = std::prev(iter)->params_[0].t0_ + std::prev(iter)->params_[0].T_;
                        for (auto& p : iter->params_) {
                            p.t0_ = t0;
                        }
                    }
                }
                break;
            }
        }

        // 逆向循环计算 va vc 等
        for (auto iter = std::prev(end_iter); iter != begin_iter; --iter) {
            for (Size i = 0; i < iter->params_.size(); ++i) {
                if (iter != std::prev(end_iter)) {
                    iter->params_[i].vb_ = std::next(iter)->params_[i].va_;
                }
                s_scurve_cpt_vavc(iter->params_[i]);
            }
        }

        // 计算起始节点的 va vc 等
        for (Size i = 0; i < begin_iter->params_.size(); ++i) {
            if (begin_iter != std::prev(end_iter)) {
                begin_iter->params_[i].vb_ = std::next(begin_iter)->params_[i].va_;
            }
            s_scurve_cpt_vavc(begin_iter->params_[i]);
        }

        

        return 0;
    }

    // 计算指定时间处的 p v a j
    auto ARIS_API s_scurve_at(const SCurveParam& param, LargeNum t, LargeNum* p_out, double* v_out, double* a_out, double* j_out)noexcept->void {
        const double va = std::max(param.va_, 0.0);
        const double vb_max = param.vb_max_;
        const double vc_max = param.vc_max_;
        const double a = param.a_;
        const double j = param.j_;
        const double T = param.T_;
        const double Ta = param.Ta_;
        const double Tb = param.Tb_;
        const LargeNum pa = param.pa_;
        const LargeNum pb = param.pb_;
        const double vb = param.vb_;
        const double vc = param.vc_;
        const int    mode = param.mode_;
        const double pt = param.pb_ - param.pa_;

        double t_ = t - param.t0_;

        LargeNum p_;
        double v_, a_, j_;
        // %CASE B
        if (mode == 1) {
            const double Ta = param.Ta_ - std::min(param.Ta_, param.Tb_);
            const double Tb = param.Tb_ - std::min(param.Ta_, param.Tb_);
            const double lower_ratio = (param.T_ - Ta - Tb) < 1e-9 ? 1.0 : (param.T_ - param.Ta_ - param.Tb_) / (param.T_ - Ta - Tb);
            const double a = param.a_ * lower_ratio;
            const double j = param.j_ * lower_ratio * lower_ratio;


            if (t_ < Ta) {
                p_ = pa + va * t_;
                v_ = va;
                a_ = 0.0;
                j_ = 0.0;
            }
            else if (t_ < T - Tb) {
                double Tacc = T - Ta - Tb;
                double si = aris::dynamic::s_sgn2(vb - va);
                t_ = t_ - Ta;
                if (Tacc >= 2.0 * a / j) {
                    if (t_ < a / j) {
                        p_ = pa + (va * Ta + va * t_ + si / 6.0 * j * t_ * t_ * t_);
                        v_ = va + si / 2.0 * j * t_ * t_;
                        a_ = si * j * t_;
                        j_ = si * j;
                    }
                    else if (t_ < (Tacc - a / j)) {
                        p_ = pa + (va * Ta + va * a / j + si / 6.0 * a * a * a / j / j + (va + si / 2.0 * a * a / j) * (t_ - a / j) + si * a / 2.0 * (t_ - a / j) * (t_ - a / j));
                        v_ = va - (a * a * si) / (2 * j) + a * si * t_;
                        a_ = a * si;
                        j_ = 0.0;
                    }
                    else {
                        p_ = pa + (va * Ta + (va + vb) / 2.0 * Tacc - (vb * (Tacc - t_) - si / 6.0 * j * (Tacc - t_) * (Tacc - t_) * (Tacc - t_)));
                        v_ = vb - (j * si * (Tacc - t_) * (Tacc - t_)) / 2.0;
                        a_ = si * j * (Tacc- t_);
                        j_ = -si * j;
                    }
                }
                else {
                    if (t_ < Tacc / 2.0) {
                        p_ = pa + (va * Ta + va * t_ + si / 6.0 * j * t_ * t_ * t_);
                        v_ = (j * si * t_ * t_) / 2.0 + va;
                        a_ = j * si * t_;
                        j_ = j * si;
                    }
                    else {
                        p_ = pa + (va * Ta + (va + vb) / 2.0 * Tacc - (vb * (Tacc - t_) - si / 6.0 * j * (Tacc - t_) * (Tacc - t_) * (Tacc - t_)));
                        v_ = vb - (j * si * (Tacc - t_) * (Tacc - t_)) / 2.0;
                        a_ = j * si * (Tacc- t_);
                        j_ = -j * si;
                    }
                }
            }
            else {
                p_ = pb - (vb * (T - t_));
                v_ = vb;
                a_ = 0.0;
                j_ = 0.0;
            }
        }
        else {
            //%CASE A
            if (t_ < Ta) {
                double si = aris::dynamic::s_sgn2(vc - va);

                if (Ta >= 2.0 * a / j) {
                    if (t_ < a / j) {
                        p_ = pa + (va * t_ + si / 6.0 * j * t_ * t_ * t_);
                        v_ = (j * si * t_ * t_) / 2.0 + va;
                        a_ = j * si * t_;
                        j_ = j * si;
                    }
                    else if (t_ < (Ta - a / j)) {
                        p_ = pa + (va * a / j + si / 6.0 * a * a * a / j / j + (va + si / 2.0 * a * a / j) * (t_ - a / j) + si * a / 2 * (t_ - a / j) * (t_ - a / j));
                        v_ = va - (a * a * si) / (2.0 * j) + a * si * t_;
                        a_ = a * si;
                        j_ = 0.0;
                    }
                    else {
                        p_ = pa + ((va + vc) / 2.0 * Ta - (vc * (Ta - t_) - si / 6.0 * j * (Ta - t_) * (Ta - t_) * (Ta - t_)));
                        v_ = vc - (j * si * (Ta - t_) * (Ta - t_)) / 2.0;
                        a_ = j * si * (Ta - t_);
                        j_ = -j * si;
                    }
                }
                else {
                    if (t_ < Ta / 2.0) {
                        p_ = pa + (va * t_ + si / 6.0 * j * t_ * t_ * t_);
                        v_ = (j * si * t_ * t_) / 2.0 + va;
                        a_ = j * si * t_;
                        j_ = j * si;
                    }
                    else {
                        p_ = pa + ((va + vc) / 2.0 * Ta - (vc * (Ta - t_) - si / 6.0 * j * (Ta - t_) * (Ta - t_) * (Ta - t_)));
                        v_ = vc - (j * si * (Ta - t_) * (Ta - t_)) / 2;
                        a_ = j * si * (Ta - t_);
                        j_ = -j * si;
                    }
                        
                }
            }
            else if (t_ < T - Tb) {
                p_ = pa + ((va + vc) / 2.0 * Ta + vc * (t_ - Ta));
                v_ = vc;
                a_ = 0.0;
                j_ = 0.0;
            }
            else {
                double si = aris::dynamic::s_sgn2(vb - vc);
                if (Tb >= 2.0 * a / j) {
                    if (T - t_ < a / j) {
                        p_ = pb + (-vb * (T - t_) + si / 6.0 * j * (T - t_) * (T - t_) * (T - t_));
                        v_ = vb - (j * si * (T - t_) * (T - t_)) / 2.0;
                        a_ = j * si * (T - t_);
                        j_ = -j * si;
                    }
                    else if (T - t_ < (Tb - a / j)) {
                        p_ = pb + (-vb * a / j + si / 6.0 * a * a * a / j / j - (vb - si / 2.0 * a * a / j) * (T - t_ - a / j) + si * a / 2.0 * (T - t_ - a / j) * (T - t_ - a / j));
                        v_ = vb + (a * a * si) / (2 * j) - T * a * si + a * si * t_;
                        a_ = a * si;
                        j_ = 0.0;
                    }
                    else {
                        p_ = pb + (-(vc + vb) / 2.0 * Tb + (vc * (Tb - T + t_) + si / 6.0 * j * (Tb - T + t_) * (Tb - T + t_) * (Tb - T + t_)));
                        v_ = vc + (j * si * (Tb - T + t_) * (Tb - T + t_)) / 2.0;
                        a_ = (Tb - T + t_) * j * si;
                        j_ = j * si;
                    }
                }
                else {
                    if (T - t_ < Tb / 2.0) {
                        p_ = pb + (-vb * (T - t_) + si / 6.0 * j * (T - t_) * (T - t_) * (T - t_));
                        v_ = vb - (j * si * (T - t_) * (T - t_)) / 2.0;
                        a_ = (T - t_) * j * si;
                        j_ = -j * si;
                    }
                    else {
                        p_ = pb + (-(vc + vb) / 2.0 * Tb + vc * (Tb - T + t_) + si / 6.0 * j * (Tb - T + t_) * (Tb - T + t_) * (Tb - T + t_));
                        v_ = vc + (j * si * (Tb - T + t_)* (Tb - T + t_)) / 2.0;
                        a_ = (Tb - T + t_) * j * si;
                        j_ = j * si;
                    }
                }
            }
        }
        if (p_out) *p_out = p_;
        if (v_out) *v_out = v_;
        if (a_out) *a_out = a_;
        if (j_out) *j_out = j_;
    }
}
