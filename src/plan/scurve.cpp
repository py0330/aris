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
    auto s_scurve_cpt_T_upper(const SCurveParam& param) -> double {
        const double va = param.va_;
        const double a = param.a_;
        const double j = param.j_;
        const double pt = param.pb_ - param.pa_;

        double Z1 = a * a / j;
        double pacc = va - 1.5 * Z1 > 0 ? 0.5 * (Z1 / 2 + va) * (Z1 / 2 + va) / a : 4.0 / 3.0 * va * safe_sqrt(2.0 / 3.0 * va / j);
        if (pacc <= pt)
            return std::numeric_limits<double>::infinity();
        else {
            //% 计算vb
            //% 【条件1】 加速度正好可以达到a时，所前进的长度
            //% 此时 vb = - a^2/j + va
            //% 前进时间 t = (va-vb)/a+a/j
            //% 前进长度 l = t*(va+vb)/2 = (2*a*va)/j - a*a*a/j/j
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
                return s_acc_time(va, vb, a, j);
            }
            else {
                //% 此时有匀速段
                //% 前进时间为：t = (va-vb)/a+a/j;
                //% 前进长度为：
                //% l = t*(va+vb)/2
                //%   = - vb^2/(2*a) + (a*vb)/(2*j) + (va*(a/j + va/a))/2
                //% vb 需求解一元二次方程 【
                //%       1,
                //%       (va - a*(a/j + va/a)),
                //%       2*(pb-pa)*a- a*va*(a/j + va/a)
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
    auto s_scurve_cpt_T_range(SCurveParam& param) -> std::tuple<double, double> {
        
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
        if (va_upper < param.va_below_) {
            return std::make_tuple(-1.0, -1.0);
        }

        param.va_ = param.va_below_;
        double T_upper = s_scurve_cpt_T_upper(param);
        param.va_ = va_upper;
        double T_below = s_scurve_cpt_T_below(param);

        return std::make_tuple(T_upper, T_below);
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
        for (aris::Size i = 0; i < begin->params_.size(); ++i) {
            auto next_iter = std::next(begin);
            
            // 计算 vb range
            s_scurve_cpt_vb_range(begin->params_[i]);

            // 更新 vb
            std::next(begin)->params_[i].va_ = begin->params_[i].vb_;
            std::next(begin)->params_[i].va_upper_ = begin->params_[i].vb_upper_;
            std::next(begin)->params_[i].va_below_ = begin->params_[i].vb_below_;

            // 计算 Tmax Tmin
            std::tie(T_uppers[i], T_belows[i]) = s_scurve_cpt_T_range(std::next(begin)->params_[i]);
        }

        double Tmin_all = *std::max_element(T_belows.begin(), T_belows.end());
        double Tmax_all = *std::min_element(T_uppers.begin(), T_uppers.end());

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
            //std::cout << "trajectory count:" << count_ << std::endl;
            //if (count_ >= 66)
            //    std::cout << "debuging" << std::endl;

            count_++;
#endif

            // STEP 1 : 计算全部末端的最大最小时间
            std::vector<double> Tmaxs(iter->params_.size()), Tmins(iter->params_.size());
            for (Size i = 0; i < iter->params_.size(); ++i) {
                // 设置正确的 pa 和 va
                if (iter != begin_iter) {
                    iter->params_[i].va_ = std::prev(iter)->params_[i].vb_;
                    iter->params_[i].va_upper_ = std::prev(iter)->params_[i].vb_upper_;
                    iter->params_[i].va_below_ = std::prev(iter)->params_[i].vb_below_;
                }

                // 计算 Tmax Tmin
                std::tie(Tmaxs[i], Tmins[i]) = s_scurve_cpt_T_range(iter->params_[i]);
            }

            double Tmin_all = *std::max_element(Tmins.begin(), Tmins.end());
            double Tmax_all = *std::min_element(Tmaxs.begin(), Tmaxs.end());

            Tmin_all = std::max(Tmin_all, T_min);

            // 若起始速度过大，有可能无法规划成功 //
            if (Tmin_all > Tmax_all){
                THROW_FILE_LINE("FAILED TO REPLAN, Tmin:" + std::to_string(Tmin_all)+ "  Tmax:" + std::to_string(Tmax_all));
            }

            // STEP 2 : 基于 T_below 求得 T_upper 上限
            double T_upper = Tmax_all;
            double T_below = std::max(Tmin_all, T_min);

            if (T_upper == std::numeric_limits<double>::infinity()) {
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

            // STEP 3 : 二分法求解最优的可行时间
            double diff = std::abs(T_upper - T_below);
            double diff_last = 10 * diff;

            while (diff < diff_last) {
                diff_last = diff;
                double T_next = (T_upper + T_below) / 2;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //std::cout << "  Tnext:" << T_next << std::endl;              
#endif
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
                        //p.t0_count_ = std::prev(iter)->params_[0].t0_count_ + std::lround((t0 - std::fmod(t0, 1000.0))/1000.0);
                    }
                }

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


    auto s_scurve_plan_eliminate_optimization(double vb1, SCurveParam& param1)->void
    {
        double T_va_to_vb = s_acc_time(vb1, param1.va_, param1.a_, param1.j_);
        double l1 = param1.pb_ - param1.pa_;
        if (l1 > (param1.T_ - T_va_to_vb) * std::max(vb1, param1.va_) + T_va_to_vb * (vb1 + param1.va_) / 2) {
            // mode 0 
            param1.mode_ = 0;
            param1.vb_ = vb1;

            auto vc1 = newton_raphson_binary_search([&param1, l1](double vc)->double {
                double Ta = s_acc_time(param1.va_, vc, param1.a_, param1.j_);
                double Tb = s_acc_time(param1.vb_, vc, param1.a_, param1.j_);
                return Ta * (param1.va_ + vc) / 2 + Tb * (param1.vb_ + vc) / 2 + (param1.T_ - Ta - Tb) * vc - l1;
                }, std::max(vb1, param1.va_), param1.vc_);

            param1.vc_ = vc1;
            param1.Ta_ = s_acc_time(param1.va_, vc1, param1.a_, param1.j_);
            param1.Tb_ = s_acc_time(param1.vb_, vc1, param1.a_, param1.j_);
        }
        else {
            // mode 1
            param1.mode_ = 1;
            param1.vb_ = vb1;

            double Tc = s_acc_time(param1.va_, param1.vb_, param1.a_, param1.j_);
            double va = vb1;
            double v_avg = (param1.T_ - Tc) > 1e-12 ? (l1 - Tc * (param1.va_ + param1.vb_) / 2) / (param1.T_ - Tc) : (param1.va_ + param1.vb_) / 2;
            param1.Ta_ = std::abs(param1.va_ - param1.vb_) > 1e-12 ? std::abs((v_avg - param1.vb_) / (param1.va_ - param1.vb_)) * (param1.T_ - Tc) : (param1.T_ - Tc) / 2;
            param1.Tb_ = param1.T_ - param1.Ta_ - Tc;
            param1.vc_ = v_avg;
        }
    }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
    auto s_compute_scurve_Tmax_Tmin(const SCurveParam& param, double T_min_set)->std::tuple<double, double>;
#endif

    // 根据起始条件，终止位置，总时间来计算 S 曲线
    //
    // param 中的以下信息为输入：
    // - pb
    // - vc_max_
    // - vb_max_
    // - a
    // - j
    // - pa
    // - va
    // - T
    // 
    // 以下信息会改变：
    // - vb_
    // - vc_
    // - Ta_
    // - Tb_
    // - mode_
	auto s_compute_scurve_node(SCurveParam& param)->void {
        const double va     = std::max(param.va_, 0.0);
        const double vb_max = param.vb_max_;
        const double vc_max = param.vc_max_;
        const double a      = param.a_;
        const double j      = param.j_;
        const double pt     = param.pb_ - param.pa_;
        const double T      = param.T_;

        const double Z1 = a*a/j;
        const double Z2 = T*T*j;

        const double l_cons = 2000000.0 * std::max(pt, 1.0) * std::numeric_limits<double>::epsilon();
        const double t_cons = 2000000.0 * std::max(T, 1.0) * std::numeric_limits<double>::epsilon();

        double T_va_to_max_v = s_acc_time(va, vc_max, a, j);
        double T_0_to_va = s_acc_time(va, 0, a, j);

        if (pt < std::numeric_limits<double>::epsilon() * 100) {
            param.vb_ = 0.0;
            param.vc_ = 0.0;
            param.mode_ = 1;
            param.Ta_ = param.T_ / 2.0;
            param.Tb_ = param.T_ / 2.0;
            return;
        }



        double& Ta = param.Ta_;
        double& Tb = param.Tb_;
        double& vb = param.vb_;
        double& vc = param.vc_;
        int& mode = param.mode_;
        double Tc, l, la, lb, lc, v_upper, v_below;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        //% ------------------ l1 --------------------- %
        //% 全力减速
        Ta = std::min(T, T_0_to_va);
        vb = std::max(s_acc_vend(va,-a,-j,Ta), 0.0);
        l  = Ta * (va + vb) / 2.0;

        double l1 = l;

        if (pt < l - 1e-10) {
            THROW_FILE_LINE("failed to compute trajectory : l1");
        }
#endif // DEBUG_ARIS_PLAN_TRAJECTORY

        //% ------------------ l2 --------------------- %
        //% vb为0，无需加速即可完成
        //% 此时 0 < v < va, 因此 Ta 段匀速va，Tb段匀速0，Tc段减速到0 
        l = -1;
        if (T >= T_0_to_va) {
            Tc = T_0_to_va;
            Ta = T - Tc;
            l = Ta * va + Tc * va / 2;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l2 = l;
#endif
        if (pt < l) {
            Tc = T_0_to_va;
            param.Ta_ = (pt - Tc * va / 2) / va;
            param.Tb_ = T - Ta - Tc;
            param.vc_ = (T - Tc) > t_cons ? Ta / (T - Tc) * va : 0.0;
            param.vb_ = 0;
            param.mode_ = 1;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * va + (T - Ta - Tb) * va /2 - pt) > l_cons
                ) {
                THROW_FILE_LINE("ERROR l2");
            }
#endif
            return;
        }

        //% ------------------ l3 --------------------- %
        //% vb为0，a段达不到最大加速度，b段达不到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb <= T
        //% B. v - va  <= a^2/j && v <= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 首先根据条件 A 和 B 确定 v 的取值范围，然后再计算 l
        //% Ta = 2 * sqrt((v-va)/j)
        //% Tb = 2 * sqrt(v/j)
        //% 带入条件 A，以下为推导对 v 的表达式：
        //% eq = (Ta+Tb)^2
        //% expand(eq)
        //% >> 8*(v/j - va/j)^(1/2)*(v/j)^(1/2) + (8*v)/j - (4*va)/j
        //% left  = 8*(v/j - va/j)^(1/2)*(v/j)^(1/2)
        //% right = T^2 + (4*va)/j - (8*v)/j
        //%
        //% 原不等式等效于 left^2 - right^2 <= 0 && right >= 0
        //%
        //% expand(left^2 - right^2)
        //% collect(left^2 - right^2,v)
        //% >> ((16*(T^2 + (4*va)/j))/j - (64*va)/j^2)*v - (T^2 + (4*va)/j)^2
        //% solve(left^2 - right^2,v)
        //% >> (T^4*j^2 + 8*T^2*j*va + 16*va^2)/(16*T^2*j)
        //% 对于不等式 left^2 - right^2 <= 0，应有：
        //% v <= (T^4*j^2 + 8*T^2*j*va + 16*va^2)/(16*T^2*j)
        //% && v <= (j*T^2)/8 + va/2
        //% 考虑其他条件应有：
        //% v_upper = min(max_v, a^2/j 
        //%               ,(T^4*j^2 + 8*T^2*j*va + 16*va^2)/(16*T^2*j)
        //%               ,(j*T^2)/8 + va/2)
        //% v_below = va
        v_upper = std::min(std::min(std::min(vc_max, Z1), Z2/16.0 + va/2.0 + va*va/Z2), Z2/8.0 + va/2.0);
        v_below = va;
        l = -1;
        if (v_below <= v_upper) {
            vc = v_upper;
            Ta = s_acc_time(va, vc, a, j);
            Tb = s_acc_time(0, vc, a, j);
            Tc = T - Ta - Tb;
            l = (va + vc) / 2 * Ta + vc / 2 * Tb + vc * Tc;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l3 = l;
#endif
        if (pt < l) {
            //% syms va j Ta pt T f(Ta)
            //% v  = va + j*Ta*Ta/4
            //% la = (v+va)/2*Ta
            //% Tb = sqrt(4*va/j + Ta*Ta)
            //% lb = Tb*v/2
            //% lc = (T - Ta - Tb)*v
            //%
            //% l = la+lb+lc
            //%   = la + T*v - Ta*v -Tb*v/2
            param.Ta_ = newton_raphson_binary_search([va, j, pt, T](double Ta)->double {
                return T * va - (Ta * Ta * Ta * j) / 8
                    - (va / 2 + Ta * Ta * j / 8) * safe_sqrt(Ta * Ta + 4 * va / j)
                    + (T * Ta*Ta * j) / 4 - pt;
                }
            , 0, std::min(std::min(s_acc_time(va, vc, a, j), T / 2.0), T - T_0_to_va));
            param.Tb_ = safe_sqrt(4.0 * va / j + Ta * Ta);
            param.vc_  = va + j*Ta*Ta/4.0;
            param.vb_ = 0;
            param.mode_ = 0;
            
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * vc / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                ) {

                v_upper = std::min(std::min(std::min(vc_max, Z1), Z2 / 16.0 + va / 2.0 + va * va / Z2), Z2 / 8.0 + va / 2.0);
                v_below = va;
                l = -1;
                if (v_below <= v_upper) {
                    vc = v_upper;
                    Ta = s_acc_time(va, vc, a, j);
                    Tb = s_acc_time(0, vc, a, j);
                    Tc = T - Ta - Tb;
                    l = (va + vc) / 2 * Ta + vc / 2 * Tb + vc * Tc;
                }

                param.Ta_ = newton_raphson_binary_search([va, j, pt, T](double Ta)->double {
                    return T * va - (Ta * Ta * Ta * j) / 8
                        - (va / 2 + Ta * Ta * j / 8) * safe_sqrt(Ta * Ta + 4 * va / j)
                        + (T * Ta * Ta * j) / 4 - pt;
                    }
                , 0, std::min(std::min(s_acc_time(va, vc, a, j), T / 2.0), T - T_0_to_va));

                THROW_FILE_LINE("ERROR l3");
            }
#endif
            return;
        }

        //% ------------------ l4 --------------------- %
        //% vb为0，a段达不到最大加速度，b段可达到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb <= T
        //% B. v - va  <= a^2/j && v >= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 首先根据条件 A 和 B 确定 v 的取值范围，然后再计算 l
        //% Ta = 2 * sqrt((v-va)/j)
        //% Tb = v/a + a/j
        //% 带入条件 A, 有：
        //% Ta^2 - (T-Tb)^2 <= 0 && Tb <= T
        //% 展开有：
        //% -v^2 + k1*v + k0 <=0 && v  <= T*a-a^2/j 
        //% 其中：
        //% k1 = (2*T*a + 2*a^2/j)
        //% k0 = -a^2*((T - a/j)^2 + (4*va)/j)
        //%
        //% 可以看出，其极值点为 a^2/j + T*a，然而若v 大于此值，必有 Tb > T
        //% 故而取其较小的根 x1
        //% 因此有 v <= x1
        //% 进一步的，根据条件 B，可以得到v的取值范围：
        //%
        //% v_upper = min(x1, va + a^2/j, max_v, T*a-a^2/j)
        //% v_below = a^2/j
        {
            double k1 = 2 * (T * a + Z1);
            double k0 = -a * a * ((4 * va) / j + T * T + Z1 / j - (2 * T * a) / j);
            double x1 = (k1 - safe_sqrt(k1 * k1 + 4 * k0)) / 2; // k0为 - 1

            v_upper = std::min(std::min(std::min(x1, va + Z1), vc_max), T * a - Z1);
            v_below = std::max(Z1, va);
        }
        l = -1;
        if (v_upper >= v_below) {
            vc = v_upper;
            Ta = s_acc_time(va, vc, a, j);
            Tb = s_acc_time(0, vc, a, j);
            Tc = T - Ta - Tb;
            l = (va + vc) / 2 * Ta + vc / 2 * Tb + vc * Tc;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l4 = l;
#endif
        if (pt < l) {
            //% clear
            //% syms va j Ta pt T a f(Ta)
            //% v  = va + j*Ta*Ta/4
            //% la = (v+va)/2*Ta
            //% Tb = (v + a^2/j)/a
            //% lb = Tb*v/2
            //% lc = (T - Ta - Tb)*v
            //%
            //% l = la+lb+lc
            //%
            //% collect(l-pt,Ta)
            //%
            //% k4 = -j^2/32
            //% k3 = -j/8
            //% k2 = (j*(a^2/j + va))/8 - (j*va)/8 - (j*(a^2/j - T + va))/4
            //% k0 = (va*(a^2/j + va))/2 - va*(a^2/j - T + va) - pt
            //%
            //% 考虑4次方程过于复杂，这里用迭代方法
            double k4 = -j*j / (32.0 * a);
            double k3 = -j / 8.0;
            double k2 = (T * j) / 4.0 - a / 8.0 - (j * va) / (4.0 * a);
            double k0 = T * va - pt - va * va / (2.0 * a) - (a * va) / (2.0 * j);
            //% 求其上下界
            //% syms f(Ta) g(Ta)
            //% f(Ta) = k4*Ta^4 + k3*Ta^3 + k2*Ta^2 + k0
            //% g(Ta) = 4*k4*Ta^3 + 3*k3*Ta^2 + 2*k2*Ta
            //% solve(g,Ta)
            //%
            //% 可得 f 的三个极值点：
            //% r0 = 0
            //% r1 = (-3*a + (a^2 + (T*j*a)*16 - (j*va)*16)^(1/2))/(2*j);
            //% r2 = (-3*a - (a^2 + (T*j*a)*16 - (j*va)*16)^(1/2))/(2*j);
            //% 
            //% 分析极值点分布，有：
            //% r1 = (-3*a + (a^2 + (T*j*a)*16 - (j*va)*16)^(1/2))/(2*j)
            //%    = (-3*a + (a^2 + 16*j*(T*a-va))^(1/2))/(2*j)
            //%
            //% 由于b段必定可达加速度a，因此有 T >= Tb = v/a+a/j >= va/a+a/j
            //% 因此有：
            //% r1 =  (-3*a + (a^2 + 16*j*(T*a-va))^(1/2))/(2*j)
            //%    >= (-3*a + (17*a^2)^(1/2))/(2*j)
            //%    >  0
            //% 而显然 r2 < 0
            //% 因此其极值点分布为：
            //% r2 < r0 = 0 < r1
            //%
            //% 下求 f(0)：
            //% f(0) =  T*va - pt - va^2/(2*a) - (a*va)/(2*j)
            //%      =  -pt + va*(Ta + Tb + Tc - (va/a + a/j)/2)
            //%      =  -pt + va * Ta + va * (Tb-(va/a + a/j)/2) + va * Tc
            //%      <= -pt + (va+v)/2*Ta + va*(v/a-va/(2*a)+a/(2*j)) + v*Tc
            //%      =  -pt + la + lc + 1/(2*a)*(2*va*v - va*va + va*a*a/j)
            //%      =  -pt + la + lc + 1/(2*a)*(v*v-(v*v-2*v*va +va*va) + va*a*a/j)
            //%      =  -pt + la + lc + 1/(2*a)*(v*v + va*a*a/j)
            //%      <= -pt + la + lc + 1/(2*a)*(v*v + v*a*a/j)
            //%      =  -pt + la + lc + v*(v/a + a/j)/2
            //%      =  -pt + la + lc + lb
            //%      =  0
            //%
            //% 因此可用 newton_raphson 方法搜索 [0,r1] 内的取值
            double Ta_below = 0;
            double Ta_upper = (-3.0*a + safe_sqrt(a*a + T*j*a*16.0 - j*va*16.0))/(2.0*j);
            param.Ta_ = newton_raphson_binary_search([k4,k3,k2,k0](double x)->double {return x*x*(x*(x*k4 + k3) + k2) + k0; }, Ta_below, Ta_upper);
            param.vc_ = va + j * Ta * Ta / 4.0;
            param.Tb_ = (vc + Z1) / a;
            param.vb_ = 0.0;
            param.mode_ = 0;

            // adjust //
            param.Ta_ = std::max(0.0, param.Ta_);
            param.Tb_ = std::max(0.0, param.Tb_);
            if (param.Ta_ + param.Tb_ > T + t_cons) {
                double ratio = T / (param.Ta_ + param.Tb_);
                param.Ta_ *= ratio;
                param.Tb_ *= ratio;
            }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * vc / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                ) {
                double l = k4 * Ta * Ta * Ta * Ta + k3 * Ta * Ta * Ta + k2 * Ta * Ta + k0;

                std::cout << "debug -----------------------------" << std::setprecision(18) << std::endl;
                auto [Tmax, Tmin] = s_compute_scurve_Tmax_Tmin(param, 0.001);
                std::cout << "Tmax:" << Tmax << std::endl;
                std::cout << "Tmin:" << Tmin << std::endl;

                std::cout << "Ta:" << Ta << std::endl;
                std::cout << "Tb:" << Tb << std::endl;
                std::cout << "T :" << T << std::endl;
                std::cout << "T - Ta - Tb:" << T - Ta - Tb << std::endl;
                std::cout << "l :" << l << std::endl;
                std::cout << "pt:" << pt << std::endl;
                std::cout << "l-pt:" << l - pt << std::endl;

                std::cout << "Ta_below:" << Ta_below << std::endl;
                std::cout << "Ta_upper:" << Ta_upper << std::endl;

                std::cout << "k0:" << k0 << std::endl;
                std::cout << "k2:" << k2 << std::endl;
                std::cout << "k3:" << k3 << std::endl;
                std::cout << "k3:" << k4 << std::endl;
                std::cout << "a :" << param.a_ << std::endl;
                std::cout << "j :" << param.j_ << std::endl;
                std::cout << "va:" << param.va_ << std::endl;
                std::cout << "vc:" << param.vc_ << std::endl;
                
                std::cout << "vb:" << param.vb_ << std::endl;
                std::cout << "vc max:" << param.vc_max_ << std::endl;
                std::cout << "vb max:" << param.vb_max_ << std::endl;


                THROW_FILE_LINE("ERROR l4");
            }
#endif

            return;
        }

        //% ------------------ l5 --------------------- %
        //% vb为0，a段达不到最大加速度，b段可达到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb <= T
        //% B. v - va  >= a^2/j && v >= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 首先根据条件 A 和 B 确定 v 的取值范围，然后再计算 l
        //% Ta = (v-va)/a + a/j
        //% Tb = v/a + a/j
        //% 带入条件 A, 有：
        //% v <= - a^2/j + (T*a)/2 + va/2
        //%
        //% v_upper = min(max_v, - a^2/j + (T*a)/2 + va/2)
        //% v_below = va + a^2/j
        v_upper = std::min(vc_max, - Z1 + (T*a)/2 + va/2);
        v_below = va + Z1;
        l = -1;
        if (v_below <= v_upper) {
            vc = v_upper;
            Ta = (vc - va) / a + a / j;
            Tb = vc / a + a / j;
            Tc = T - Ta - Tb;
            l = Ta * (vc + va) / 2 + Tb * vc / 2 + Tc * vc;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l5 = l;
#endif
        if (pt < l) {
            //% clear
            //% syms va j Ta pt T a f(Ta)
            //% v  = va + Ta*a - a^2/j
            //% la = (v+va)/2*Ta
            //% Tb = (v + a^2/j)/a
            //% lb = Tb*v/2
            //% lc = (T - Ta - Tb)*v
            //%
            //% l  = la+lb+lc-pt
            //%
            //% collect(l,Ta)
            //%
            //% k2 = -a
            //% k1 = (a*(T - va/a) + a^2/j)
            //% k0 = (T - va/a)*(va - a^2/j) + (va*(va - a^2/j))/(2*a)- pt
        
            double k2 = -a;
            double k1 = (a*T - va + Z1);
            double k0 = (va - Z1)*(T - va/a/2)- pt;
    
            param.Ta_ = (-k1 + safe_sqrt(k1*k1 - 4*k0*k2))/(2*k2);
            param.vc_ = va + Ta*a - Z1;
            param.vb_ = 0;
            param.Tb_ = (vc + Z1)/a;
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * vc / 2 + (T-Ta-Tb)*vc - pt) > l_cons
                ) {
                THROW_FILE_LINE("ERROR l5");
            }
#endif
            return;
        }

        //% ------------------ l6 --------------------- % 
        //% vb不为0，达不到max_v，a段达不到最大加速度，b段可达到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb =  T
        //% B. v - va  <= a^2/j && v - vb >= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 条件 A   可得 Ta  = T - Tb
        //%            => Ta <= T - 2*a/j   (因为 Tb >= 2*a/j)
        //%
        //% 条件 B.1 可得 Ta <= 2*a/j
        //% 条件 B.2 可得 va + j*Ta*Ta/4 >= a^2/j
        //%            => Ta >= 2*sqrt(max(0, a^2/j - va)/j)
        //% 条件 B.3 可得 Ta <= T_va_to_max_v
        //% 条件 B.4 可得 Ta >= 0  (包含在B.2中)
        double Ta_upper = std::min(std::min(T_va_to_max_v, 2.0*a/j), T - 2.0*a/j);
        double Ta_below = 2.0* safe_sqrt((Z1 - va)/j);
        if (Ta_upper >= Ta_below) {
            Ta = Ta_upper;
            vc = va + j * Ta * Ta / 4.0;
            la = Ta * (vc + va) / 2.0;
            Tb = T - Ta;
            vb = vc - Tb * a + Z1;
            lb = Tb * (vc + vb) / 2.0;
            l = la + lb;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l6 = l;
#endif
        if (pt < l) {
            //%   clear
            //%   syms va j Ta a T
            //%   v  = va + j*Ta*Ta/4
            //%   la = j/8*Ta^3 + va*Ta
            //%   Tb = T-Ta
            //%   vb = v - Tb*a + a^2/j;
            //%   lb = Tb*(v+vb)/2
            //%   l  = la + lb
            //%
            //%   【result】:
            //%   带入方程la + lb = pt
            //%   可得：
            //%   k3*Ta^3 + k2*Ta^2 + k1*Ta + k0
            //%   其中：
            //%   k3 = - j/8
            //%   k2 = (T*j)/4 - a/2
            //%   k1 = - a^2/(2*j) + T*a
            //%   k0 = (T*(a^2/j - T*a + 2*va))/2 - pt
            //%
            //%   【condition】:
            //%   Tb > 2*a/j
            //%   => Ta < T - 2*a/j
            //%   于是：
            //%   0 <= Ta <= min(T, 2*a/j, T - 2*a/j)
            //%     
            //%   计算Ta = min(T, 2*a/j, T - 2*a/j)
            //%   l3 = la + lb  
            double k3 = -j / 8;
            double k2 = (T * j) / 4 - a / 2;
            double k1 = -Z1 / 2 + T * a;
            double k0 = (T * (Z1 - T * a + 2 * va)) / 2 - pt;
            //% 选根
            //% syms f(Ta) g(Ta)
            //% f(Ta) = k3*Ta^3 + k2*Ta^2 +k1*Ta + k0
            //% g(Ta) = diff(f,Ta)
            //% solve(g,Ta)
            //% 
            //% 可得其极值：
            //% r1 = 4/3*T - 2/3*a/j
            //% r2 = -(2*a)/j
            //%
            //% 由于b段可达最大加速度，因此必有 T >= 2a/j
            //% 于是
            //% r1 >= 4/3*T - 1/3*T = T
            //% 因此其上下界为 [0,T]
            param.Ta_ = newton_raphson_binary_search([k0,k1,k2,k3](double x)->double {
                return x*(x*(k3 * x + k2) + k1) + k0;
                }, 0, T);
            param.vc_ = va + j * Ta * Ta / 4;
            param.Tb_ = T - Ta;
            param.vb_ = s_acc_vend(vc, -a, -j, Tb);
            param.mode_ = 0;

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb)*vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {
                auto [Tmax, Tmin] = s_compute_scurve_Tmax_Tmin(param, 0.001);
                THROW_FILE_LINE("ERROR l6");
            }
#endif
            return;
        }

        //
        //% ------------------ l8 -------------------- 
        //% vb不为0，达不到max_v，a段可达到最大加速度，b段可达到最大加速度
        Ta_upper = std::min(T_va_to_max_v, T - 2.0 * a / j);
        Ta_below = 2.0 * a / j;
        l=-1;
        if (Ta_upper >= Ta_below) {
            Ta = Ta_upper;
            vc = va + Ta * a - Z1;
            la = (va + vc) * Ta / 2;
            Tb = T - Ta;
            vb = s_acc_vend(vc, -a, -j, Tb);
            lb = Tb * (vc + vb) / 2;
            l = la + lb;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l8 = l;
#endif
        if (pt < l) {
            //    % syms va T a j pt Ta
            //    % v  = va + Ta*a - a^2/j
            //    % la = Ta*(va + v)/2
            //    % Tb = T-Ta
            //    % vb = v - Tb*a + a^2/j;
            //    % lb = Tb*(v+vb)/2
            //    %
            //    % 根据 la + lb = pt，有：
            //    % collect(la+lb-pt,Ta)
            //    % - a*Ta^2 + 2*T*a*Ta - pt - (T*(T*a - 2*va + a^2/j))/2
            //    %
            //    % 可得方程系数
            double k2 = -a;
            double k1 = 2.0 * T * a;
            double k0 = -pt - (T * (Z1 + T * a - 2.0 * va)) / 2.0;
            //
            //    % 选根
            //    % 其极值为：
            //    % r = k1 / (2*k2) = T
            //    % 应有 Ta < T
            //    % 故而选其较小的根
            param.Ta_ = (-k1 + safe_sqrt(k1 * k1 - 4 * k0 * k2)) / 2 / k2;
            param.vc_ = va + Ta * a - Z1;
            param.Tb_ = T - Ta;
            param.vb_ = s_acc_vend(vc, -a, -j, Tb);
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {
                THROW_FILE_LINE("ERROR l8");
            }
#endif
            return;
        }

        //% ------------------ l10 -------------------- 
        //% vb不为0，可达max_v，b段可达到最大加速度
        Ta = std::min(s_acc_time(va, vc_max, a, j),T);
        vc = vc_max;
        Tb = std::min(std::min(T - Ta, s_acc_time(0, vc_max, a, j)), 2*a/j);
        l = -1;

        if (Tb >= 2 * a / j) {
            la = (va + vc_max) * Ta / 2;
            vb = s_acc_vend(vc, -a, -j, Tb);
            lb = Tb * (vc + vb) / 2;
            Tc = T - Ta - Tb;
            lc = Tc * vc_max;
            l = la + lb + lc;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l10 = l;
#endif // DEBUG_ARIS_PLAN_TRAJECTORY
        if (pt < l) {
            double B = -a / j;
            double C = -(2.0 * la - 2.0 * pt + 2.0 * vc_max * (T - T_va_to_max_v)) / a;

            param.Tb_ = std::max((-B + safe_sqrt(B*B - 4.0 * C)) / 2.0, 0.0);
            param.Ta_ = s_acc_time(va, vc_max, a, j);
            param.vb_ = vc_max - Tb * a + Z1;
            param.vc_ = vc_max;
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {

                

                THROW_FILE_LINE("ERROR l10");
            }
#endif
            return;
        }

        //% ------------------ l7 --------------------- 
        //% vb不为0，达不到max_v，a段达不到最大加速度，b段达不到最大加速度
        //%
        //% 此时需要满足3个条件：
        //% A. Ta + Tb =  T
        //% B. v - va  <= a^2/j && v - vb <= a^2/j && v <= max_v && v >= va
        //% C. pt <= l
        //%
        //% 条件 A   可得 Ta =  T-Tb
        //%            => Ta >= T - 2*a/j   (因为Tb < 2*a/j)
        //%
        //% 条件 B.1 可得 Ta <= 2*a/j
        //% 条件 B.2 无法得到有效等式，因为vb可以为为任意值
        //% 条件 B.3 可得 Ta <= T_va_to_max_v
        //% 条件 B.4 可得 Ta >= 0
        Ta_upper = std::min(std::min(T, T_va_to_max_v), 2.0 * a / j);
        Ta_below = std::max(0.0, T - 2.0 * a / j);
        l = -1;
        if (Ta_upper - Ta_below >= 0) {
            Ta = Ta_upper;
            vc = va + j * Ta * Ta / 4;
            Tb = T - Ta;
            vb = std::max(vc - j * Tb * Tb / 4, 0.0);
            l = Ta * (va + vc) / 2.0 + Tb * (vb + vc) / 2.0;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l7 = l;
#endif
        //% 此处的cons为必须，因为有可能vb = v
        if (pt < l || T < std::min(T_va_to_max_v, 2.0*a/j)) {
            //    % v  = va + j*Ta*Ta/4
            //    % la = j/8*Ta^3 + va*Ta
            //    % Tb = T-Ta
            //    % lb = Tb*(v+vb)/2
            //    % 【result】:
            //    % 带入方程la + lb = pt
            //    % 可得：
            //    % k2*Ta^2 +k1*Ta + k0
            //    % 其中：
            //    % k2 = T*j/8
            //    % k1 = -3*T^2*j/8
            //    % k0 = pt - (T*(2*va - (T^2*j)/4))/2
            //    %
            //    % Ta = (-k1+sqrt(k1*k1-4*k0*k2))/2/k2
            double k2 = T * j / 8.0;
            double k1 = -3.0 * Z2 / 8.0;
            double k0 = pt - T * va + T * Z2 / 8.0;
            //
            //% 选根
            //% 其极值为(k1) / (2 * k2) = (3 * T) / 2
            //% 因此需选其较小的根
            param.Ta_ = (-k1 - safe_sqrt(k1 * k1 - 4.0 * k0 * k2)) / 2.0 / k2;

            // adjust //
            param.Ta_ = std::max(param.Ta_, 0.0);
            param.Ta_ = std::min(param.Ta_, param.T_);
            // adjust finished //

            param.vc_ = va + j * Ta * Ta / 4.0;
            param.Tb_ = T - Ta;
            param.vb_ = s_acc_vend(vc, -a, -j, Tb);
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
                ) {
                //double l = k2 * Ta * Ta + k1 * Ta + k0;
                std::cout << "debug -----------------------------" << std::setprecision(15) << std::endl;
                auto [Tmax, Tmin] = s_compute_scurve_Tmax_Tmin(param, 0.001);
                std::cout << "Tmax:" << Tmax << std::endl;
                std::cout << "Tmin:" << Tmin << std::endl;


                std::cout << "Ta:" << Ta << std::endl;
                std::cout << "Tb:" << Tb << std::endl;
                std::cout << "T :" << T << std::endl;
                std::cout << "T - Ta - Tb:" << T - Ta - Tb << std::endl;
                std::cout << "l :" << Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc << std::endl;
                std::cout << "pt:" << pt << std::endl;
                std::cout << "l-pt:" << l-pt << std::endl;
                std::cout << "vb:" << vb << std::endl;
                std::cout << "vb max:" << param.vb_max_ << std::endl;



                THROW_FILE_LINE("ERROR l7");
            }
#endif
            return;
        }


        //
        //% ------------------ l9 -------------------- 
        //% vb不为0，达不到max_v，a段可达到最大加速度，b段不可达到最大加速度
        Ta_upper = std::min(T, T_va_to_max_v);
        Ta_below = 2 * a / j;
        l = -1;
        if (Ta_upper >= Ta_below) {
            Ta = Ta_upper;
            vc = s_acc_vend(va, a, j, Ta);
            la = (va + vc) * Ta / 2;
            Tb = T - Ta;
            vb = s_acc_vend(vc, -a, -j, Tb);
            lb = Tb * (vc + vb) / 2;
            l = la + lb;
        }
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        double l9 = l;
#endif
        if (pt < l || T < T_va_to_max_v) {
            //    % syms va T a j pt Ta
            //    % v  = va + Ta*a - a^2/j
            //    % la = Ta*(va + v)/2
            //    % Tb = T-Ta
            //    % vb = v - j*Tb*Tb/4;
            //    % lb = Tb*(v+vb)/2
            //    %
            //    % 根据 la + lb = pt，有：
            //    % collect(la+lb-pt,Ta)
            //    % (j*Ta^3)/8 + (- a/2 - (3*T*j)/8)*Ta^2 + ((T^2*j)/8 + a^2/(2*j) + (T*(2*a + (T*j)/2))/2)*Ta - pt - (T*((T^2*j)/4 - 2*va + (2*a^2)/j))/2
            //    %
            //    % 可得方程系数
            double k3 = j / 8.0;
            double k2 = -a / 2.0 - 3.0 / 8.0 * T * j;
            double k1 = Z2 * 3.0 / 8.0 + Z1 / 2.0 + T * a;
            double k0 = -pt - (T * (Z2 / 8.0 + Z1 - va));
            //    
            //    % 选根
            //    % syms f(Ta) g(Ta)
            //    % f(Ta) = k3*Ta^3 + k2*Ta^2 +k1*Ta + k0
            //    % g(Ta) = diff(f,Ta)
            //    % solve(g,Ta)
            //    %
            //    % 得到:
            //    %
            //    % r1 = T + (2*a)/(3*j)
            //    % r2 = T + (2*a)/j
            //    %
            //    % 因为必有 2*a/j <= Ta <= T, 因此其上下界为：
            //    % T_below = 2*a/j
            //    % T_upper = T
            //    
            param.Ta_ = newton_raphson_binary_search([k0, k1, k2, k3](double x) ->double {
                return x * (x * (k3 * x + k2) + k1) + k0;
                }, 2.0 * a / j, T);
            param.vc_ = va + Ta * a - Z1;
            param.Tb_ = std::max(T - Ta, 0.0);
            param.vb_ = s_acc_vend(vc, -a, -j, Tb);
            param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            //% error %
            if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
                || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
                || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-9, 1e-9)
                ) {
                double l = Ta * (Ta * (k3 * Ta + k2) + k1) + k0;
                param.Ta_ = newton_raphson_binary_search([k0, k1, k2, k3](double x) ->double {
                    return x * (x * (k3 * x + k2) + k1) + k0;
                    }, 2.0 * a / j, T);
                THROW_FILE_LINE("ERROR l9");
            }
#endif
            return;
        }

        //% ------------------ l11 -------------------- 
        //% vb不为0，可达max_v，b段不可达到最大加速度
        
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        Ta = T_va_to_max_v;
        la = (va + vc_max) * Ta / 2;
        Tb = 0;
        lb = 0;
        Tc = T - Ta - Tb;
        lc = Tc * vc_max;
        l = la + lb + lc;
        if (pt > l + l_cons) {
            THROW_FILE_LINE("ERROR l12");
        }
        double l11 = l;
#endif


        // 计算 //
        param.Ta_ = T_va_to_max_v;
        param.Tb_ = std::cbrt(std::max(((va + vc_max) * Ta / 2 + vc_max * (T - T_va_to_max_v) - pt) * 8 / j, 0.0));
        param.vb_ = vc_max - j * Tb * Tb / 4;
        param.vc_ = vc_max;
        param.mode_ = 0;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
        //% error %
        if (Ta < -t_cons || Tb < -t_cons || T - Ta - Tb < -t_cons
            || std::abs(Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc - pt) > l_cons
            || (vb - param.vb_max_) > std::max(param.vb_max_ * 1e-10, 1e-10)
            ) {
            std::cout << "debug -----------------------------" << std::setprecision(15) << std::endl;
            auto [Tmax, Tmin] = s_compute_scurve_Tmax_Tmin(param, 0.001);
            std::cout << "Tmax:" << Tmax << std::endl;
            std::cout << "Tmin:" << Tmin << std::endl;


            std::cout << "Ta:" << Ta << std::endl;
            std::cout << "Tb:" << Tb << std::endl;
            std::cout << "T :" << T << std::endl;
            std::cout << "T - Ta - Tb:" << T - Ta - Tb << std::endl;
            std::cout << "l :" << Ta * (va + vc) / 2 + Tb * (vb + vc) / 2 + (T - Ta - Tb) * vc << std::endl;
            std::cout << "pt:" << pt << std::endl;
            std::cout << "l-pt:" << l - pt << std::endl;
            std::cout << "vb:" << vb << std::endl;
            std::cout << "vb max:" << param.vb_max_ << std::endl;



            THROW_FILE_LINE("ERROR l11");
        }
#endif
        return;



        
	}

    // 计算 S 曲线的最大最小时间
    //
    // param 中的以下为输入：
    // - pb
    // - vc_max_
    // - vb_max_
    // - a
    // - j
    // - pa
    // - va
    // 
    // 以下不变：
    // - vb_
    // - vc_
    // - Ta_
    // - Tb_
    // - mode_
    // - T_
    //
    // 函数返回：
    // Tmax, Tmin
    auto s_compute_scurve_Tmax_Tmin(const SCurveParam& param, double T_min_set)->std::tuple<double, double> {
        const double va     = param.va_;
        const double vb_max = param.vb_max_;
        const double vc_max = param.vc_max_;
        const double a      = param.a_;
        const double j      = param.j_;
        const double pt     = param.pb_ - param.pa_;
        const double Tmin_max = T_min_set;
        if (pt < std::numeric_limits<double>::epsilon() * 100){
            if (va > std::numeric_limits<double>::epsilon() * 100) {
                return std::make_tuple<double, double>(-1.0, -1.0);
            }
            else {
                return std::make_tuple(std::numeric_limits<double>::infinity(), Tmin_max);
            }
        }

        
        double Tmax, Tmin, vb, l, v1, v2, v_upper, v_below;

        double Z1 = a * a / j;
        double T_va_to_vb = s_acc_time(va, vb_max, a, j);
        double l_va_to_vb = T_va_to_vb * (va + vb_max) / 2;

        // failed //
        if ((!std::isfinite(Z1)) || (!std::isfinite(T_va_to_vb)) || (!std::isfinite(l_va_to_vb)) || (va > vb_max && l_va_to_vb > pt))
            return std::make_tuple(-1, -1);

        double pacc = va - 1.5 * Z1 > 0 ? 0.5 * (Z1 / 2 + va) * (Z1 / 2 + va) / a : 4.0 / 3.0 * va * safe_sqrt(2.0 / 3.0 * va / j);
        if (pacc <= pt)
            Tmax = std::numeric_limits<double>::infinity();
        else {
            //% 计算vb
            //% 【条件1】 加速度正好可以达到a时，所前进的长度
            //% 此时 vb = - a^2/j + va
            //% 前进时间 t = (va-vb)/a+a/j
            //% 前进长度 l = t*(va+vb)/2 = (2*a*va)/j - a*a*a/j/j
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
                Tmax = s_acc_time(va, vb, a, j);
            }

            else {
                //% 此时有匀速段
                //% 前进时间为：t = (va-vb)/a+a/j;
                //% 前进长度为：
                //% l = t*(va+vb)/2
                //%   = - vb^2/(2*a) + (a*vb)/(2*j) + (va*(a/j + va/a))/2
                //% vb 需求解一元二次方程 【
                //%       1,
                //%       (va - a*(a/j + va/a)),
                //%       2*(pb-pa)*a- a*va*(a/j + va/a)
                //% 】
                //% 对于根来说，应当取大值，这是因为Tmax应该尽可能的小
                double B = -Z1;
                double C = 2 * pt * a - va * Z1 - va * va;
                double vb = (-B + safe_sqrt(B * B - 4 * C)) / 2;
                Tmax = s_acc_time(va, vb, a, j);
            }
        }

        if(Tmax < Tmin_max)
            return std::make_tuple(-1.0, -1.0);

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
            Tmin = s_acc_time(va, vb, a, j);
            return std::make_tuple(Tmax, Tmin);
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
            return std::make_tuple(Tmax, Tmin);
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
            double v_minus_v1 = newton_raphson_binary_search([v1,v2,j,pt](double x) {
                return safe_sqrt(x / j) * (2.0*v1 + x) + safe_sqrt((v1 - v2 + x) / j) * (v1 + v2 + x) - pt;
                }
                , 0.0, v_upper - v_below);

            double T1 = 2 * safe_sqrt(v_minus_v1 / j);
            double T2 = 2 * safe_sqrt((v1 - v2 + v_minus_v1) / j);
            Tmin = T1 + T2;
            return std::make_tuple(Tmax, Tmin);
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
        if (pt < l){
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
                return safe_sqrt(x / j) * (x + 2.0*v1) + ((v1 - v2 + x) / a + a / j) * (v1 + v2 + x) / 2 - pt;
                }
                , v_below - v1, v_upper - v1);

            double T1 = 2 * safe_sqrt(v_minus_v1 / j);
            double T2 = (v1 - v2 + v_minus_v1) / a + a / j;
            Tmin = T1 + T2;
            return std::make_tuple(Tmax, Tmin);
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

            Tmin = std::max(Tmin, Tmin_max);
            return std::make_tuple(Tmax,Tmin);
        }

        //% ------------------ l6 --------------------
        //% 可以加速到max_v，匀速运行一段时间，之后减速到max_vb
        //% v1 = max(va,vb), v2 = min(va,vb), v = max_v
        v1 = std::max(va,vb);
        v2 = std::min(va,vb);
        double v  = vc_max;
        double T1 = s_acc_time(v1,vc_max,a,j);
        double T2 = s_acc_time(v2,vc_max,a,j);
        double T3 = (pt - T1 * (v + v1)/2 - T2 * (v + v2)/2)/vc_max;
        Tmin = T1 + T2 + T3;
        return std::make_tuple(Tmax, Tmin);
	}

    // 针对相连接的 node_1 和 node_2 来优化连接处的速度和加速度
    // 1. 连接处的速度  vb1 = va2 < vc1 AND vb1 = va2 < vc2 AND vb1 = va2 < vb1_max
    //    此时需要提速，提高结束点的速度
    auto s_optimize_ajacent_nodes(SCurveParam& param1, SCurveParam& param2)->void {
        // 【以下注释暂不可用】
        // STEP 1. 计算 param1 允许的最大的 vb 
        //    vb 最大值发生的条件：
        //       a. vb = vc 时
        //       b. vb = vb_max 时
        //    下计算 vb = vc 时，vb 的最大值
        //       a.1   va 加速到 vb，有匀加速段
        //       a.1.1 va * T < l  &&  (vb-va)>a^2/j
        //             Ta = (vb-va) / a + a/j
        //             l  = Ta * (va + vb)/2 + (T-Ta)*vb
        //             =>  vb^2 + B*vb + C = 0
        //             其中  B = a^2/j - 2*T*a - 2*va
        //                   C = va^2 - (a^2*va)/j + 2*a*l
        //             
        //             于是 vb = (-B - sqrt(B^2 - 4*C))/2
        //       a.1.2 va * T >= l
        //             此时  B = - a^2/j + 2*T*a - 2*va
        //                   C = (a^2*va)/j - 2*l*a + va^2
        //       a.2   va 加速到 vb，无匀加速段
        //       a.2.1 va * T < l
        //             Ta = 2*sqrt((vb-va)/j)

        if (((param1.mode_ == 0 && param1.vc_ > param1.vb_) || (param1.mode_ == 1 && param1.va_ > param1.vb_))
            && 
            ((param2.mode_ == 0 && param2.vc_ > param2.va_) || (param2.mode_ == 1 && param2.va_ < param2.vb_))) {
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            auto p1 = param1;
            auto p2 = param2;
#endif
            
            auto vb_upper = std::min({ 
                param1.vb_max_, 
                param1.mode_ == 0 ? param1.vc_ : param1.va_,
                param2.mode_ == 0 ? param2.vc_ : param2.vb_,
                s_acc_vend(param1.va_, param1.a_, param1.j_, param1.T_), 
                s_acc_vend(param2.vb_, param2.a_, param2.j_, param2.T_) 
                });

            auto vb_lower = param1.vb_;

            double l1 = param1.pb_ - param1.pa_;
            double l2 = param2.pb_ - param2.pa_;

            auto vb1_max = newton_raphson_binary_search([&param1, l1](double vb)->double {
                double Ta = s_acc_time(param1.va_, vb, param1.a_, param1.j_);
                return Ta * (param1.va_ + vb) / 2 + vb * (param1.T_ - Ta) - l1;
                }, vb_lower, vb_upper);

            auto va2_max = newton_raphson_binary_search([&param2, l2](double va)->double {
                double Tb = s_acc_time(param2.vb_, va, param2.a_, param2.j_);
                return Tb * (param2.vb_ + va) / 2 + va * (param2.T_ - Tb) - l2;
                }, vb_lower, vb_upper);

            auto vb1 = std::min(vb1_max, va2_max);

            // 以上理论值没错，但考虑到重规划需要保证一定能成功，还需确保 Tmin < T < Tmax
            {
                double previous_va = param2.va_;

                param2.va_ = vb1;

                double Tmax, Tmin;
                std::tie(Tmax, Tmin) = s_compute_scurve_Tmax_Tmin(param2, 0.0);
                if (param2.T_ < Tmin || param2.T_ > Tmax) {
                    double va_below = previous_va;
                    double va_upper = vb1;

                    double cons = std::max(vb1, 1e-10) * 1e-10;
                    
                    // 二分法寻找最接近 vb1 的可行 va
                    double diff = va_upper - va_below;
                    double diff_last;

                    do {
                        param2.va_ = (va_below + va_upper) / 2.0;

                        std::tie(Tmax, Tmin) = s_compute_scurve_Tmax_Tmin(param2, 0.0);
                        if (Tmin <= param2.T_ && param2.T_ <= Tmax) {
                            va_below = param2.va_;
                        }
                        else {
                            va_upper = param2.va_;
                        }
                        diff_last = diff;
                        diff = va_upper - va_below;

                    } while (diff < diff_last);

                    vb1 = va_below;
                }
            }

            // param1 //
            {
                double T_va_to_vb = s_acc_time(vb1, param1.va_, param1.a_, param1.j_);
                if (l1 > (param1.T_ - T_va_to_vb) * std::max(vb1, param1.va_) + T_va_to_vb * (vb1 + param1.va_) / 2) {
                    // mode 0 
                    param1.mode_ = 0;
                    param1.vb_ = vb1;

                    auto vc1 = newton_raphson_binary_search([&param1, l1](double vc)->double {
                        double Ta = s_acc_time(param1.va_, vc, param1.a_, param1.j_);
                        double Tb = s_acc_time(param1.vb_, vc, param1.a_, param1.j_);
                        return Ta * (param1.va_ + vc) / 2 + Tb * (param1.vb_ + vc) / 2 + (param1.T_ - Ta - Tb) * vc - l1;
                        }, std::max(vb1, param1.va_), param1.vc_);

                    param1.vc_ = vc1;
                    param1.Ta_ = s_acc_time(param1.va_, vc1, param1.a_, param1.j_);
                    param1.Tb_ = s_acc_time(param1.vb_, vc1, param1.a_, param1.j_);
                }
                else {
                    // mode 1
                    param1.mode_ = 1;
                    param1.vb_ = vb1;

                    double Tc = s_acc_time(param1.va_, param1.vb_, param1.a_, param1.j_);
                    double va = vb1;
                    double v_avg = (param1.T_ - Tc) > 1e-12 ? (l1 - Tc * (param1.va_ + param1.vb_) / 2) / (param1.T_ - Tc) : (param1.va_ + param1.vb_) / 2;
                    param1.Ta_ = std::abs(param1.va_ - param1.vb_) > 1e-12 ? std::abs((v_avg - param1.vb_) / (param1.va_ - param1.vb_)) * (param1.T_ - Tc) : (param1.T_ - Tc) / 2;
                    param1.Tb_ = param1.T_ - param1.Ta_ - Tc;
                    param1.vc_ = v_avg;
                }
            }

            // param2 //
            {
                double T_va_to_vb = s_acc_time(vb1, param2.vb_, param2.a_, param2.j_);
                if (l2 > (param2.T_ - T_va_to_vb) * std::max(vb1, param2.vb_) + T_va_to_vb * (vb1 + param2.vb_) / 2) {
                    // mode 0 
                    param2.mode_ = 0;
                    param2.va_ = vb1;
                    auto vc2 = newton_raphson_binary_search([&param2, l2](double vc)->double {
                        double Ta = s_acc_time(param2.va_, vc, param2.a_, param2.j_);
                        double Tb = s_acc_time(param2.vb_, vc, param2.a_, param2.j_);
                        return Ta * (param2.va_ + vc) / 2 + Tb * (param2.vb_ + vc) / 2 + (param2.T_ - Ta - Tb) * vc - l2;
                        }, std::max(vb1, param2.vb_), param2.vc_);
                    param2.vc_ = vc2;
                    param2.Ta_ = s_acc_time(param2.va_, vc2, param2.a_, param2.j_);
                    param2.Tb_ = s_acc_time(param2.vb_, vc2, param2.a_, param2.j_);

                }
                else {
                    // mode 1
                    param2.mode_ = 1;
                    param2.va_ = vb1;

                    double Tc = s_acc_time(param2.va_, param2.vb_, param2.a_, param2.j_);
                    double va = vb1;
                    double v_avg = (param2.T_ - Tc) > 1e-12 ? (l2 - Tc * (param2.va_ + param2.vb_) / 2) / (param2.T_ - Tc) : (param2.va_ + param2.vb_) / 2;
                    param2.Ta_ = std::abs(param2.va_ - param2.vb_) > 1e-12 ? std::abs((v_avg - param2.vb_) / (param2.va_ - param2.vb_)) * (param2.T_ - Tc) : (param2.T_ - Tc) / 2;
                    param2.Tb_ = param2.T_ - param2.Ta_ - Tc;
                    param2.vc_ = v_avg;
                }
            }

#ifdef DEBUG_ARIS_PLAN_TRAJECTORY

            if (param1.Ta_ + param1.Tb_ > param1.T_ + 1e-9) {
                //THROW_FILE_LINE("TIME ERROR");
                
                std::cout << "error" << __LINE__ << std::setprecision(15) << std::endl;
                std::cout << "l :" << l1 << std::endl;
                std::cout << "Ta:" << param1.Ta_ << std::endl;
                std::cout << "Tb:" << param1.Tb_ << std::endl;
                std::cout << "T :" << param1.T_ << std::endl;
                std::cout << "va:" << param1.va_ << std::endl;
                std::cout << "vb:" << param1.vb_ << std::endl;
                std::cout << "vc:" << param1.vc_ << std::endl;
                std::cout << "md:" << param1.mode_ << std::endl;
                std::cout << "-----------------------------------------" << std::endl;
                std::cout << "Ta:" << p1.Ta_ << std::endl;
                std::cout << "Tb:" << p1.Tb_ << std::endl;
                std::cout << "T :" << p1.T_ << std::endl;
                std::cout << "va:" << p1.va_ << std::endl;
                std::cout << "vb:" << p1.vb_ << std::endl;
                std::cout << "vc:" << p1.vc_ << std::endl;
                std::cout << "md:" << p1.mode_ << std::endl;
                std::cout << "----------------------------------------------------" << std::endl;

            }
            if (param2.Ta_ + param2.Tb_ > param2.T_ + 1e-9) {
                std::cout << param2.Ta_ * (param2.va_ + param2.vc_) / 2 + param2.Tb_ * (param2.vb_ + param2.vc_) / 2 + std::abs(param2.T_ - param2.Ta_ - param2.Tb_) * param2.vc_ << std::endl;
                std::cout << l2 << std::endl;

                auto vc2 = newton_raphson_binary_search([&param2, l2](double vc)->double {
                    double Ta = s_acc_time(param2.va_, vc, param2.a_, param2.j_);
                    double Tb = s_acc_time(param2.vb_, vc, param2.a_, param2.j_);
                    return Ta * (param2.va_ + vc) / 2 + Tb * (param2.vb_ + vc) / 2 + aris::dynamic::s_sgn2(vc - param2.vb_) * (param2.T_ - Ta - Tb) * vc - l2;
                    }, vb1, param2.vc_);


                std::cout << "error" << __LINE__ << std::setprecision(15) << std::endl;
                std::cout << "l :" << l2 << std::endl;
                std::cout << "Ta:" << param2.Ta_ << std::endl;
                std::cout << "Tb:" << param2.Tb_ << std::endl;
                std::cout << "T :" << param2.T_ << std::endl;
                std::cout << "va:" << param2.va_ << std::endl;
                std::cout << "vb:" << param2.vb_ << std::endl;
                std::cout << "vc:" << param2.vc_ << std::endl;
                std::cout << "md:" << param2.mode_ << std::endl;
                std::cout << "-----------------------------------------" << std::endl;
                std::cout << "Ta:" << p2.Ta_ << std::endl;
                std::cout << "Tb:" << p2.Tb_ << std::endl;
                std::cout << "T :" << p2.T_ << std::endl;
                std::cout << "va:" << p2.va_ << std::endl;
                std::cout << "vb:" << p2.vb_ << std::endl;
                std::cout << "vc:" << p2.vc_ << std::endl;
                std::cout << "md:" << p2.mode_ << std::endl;
                std::cout << "----------------------------------------------------" << std::endl;
            }

            //if (std::abs(param2.Ta_ * (param2.va_ + param2.vc_) / 2 + param2.Tb_ * (param2.vb_ + param2.vc_) / 2 + (param2.T_ - param2.Ta_ - param2.Tb_) * param2.vc_ - l2) > 1e-12) {
            LargeNum p_out;
            aris::plan::s_scurve_at(param2, param2.T_ + param2.t0_, &p_out);
            p_out = p_out - param2.pa_;
            if(std::abs(p_out - l2) > 1e-10){
                std::cout << param2.Ta_ * (param2.va_ + param2.vc_) / 2 + param2.Tb_ * (param2.vb_ + param2.vc_) / 2 + (param2.T_ - param2.Ta_ - param2.Tb_) * param2.vc_ << std::endl;
                std::cout << "error" << __LINE__ << std::endl;
                
                aris::plan::s_scurve_at(param2, param2.T_ + param2.t0_, &p_out);
                
                std::cout << "result: " << p_out - param2.pa_ << std::endl;

                param2 = p2;

                aris::plan::s_scurve_at(param2, param2.T_ + param2.t0_, &p_out);

                std::cout << "ori   : " << p_out - param2.pa_ << std::endl;

                auto va2_max = newton_raphson_binary_search([&param2, l2](double va)->double {
                    double Tb = s_acc_time(param2.vb_, va, param2.a_, param2.j_);
                    return Tb * (param2.vb_ + va) / 2 + va * (param2.T_ - Tb) - l2;
                    }, vb_lower, vb_upper);

                double Tb = s_acc_time(param2.vb_, va2_max, param2.a_, param2.j_);
                std::cout << Tb * (param2.vb_ + va2_max) / 2 + va2_max * (param2.T_ - Tb) - l2 << std::endl;

                double T_va_to_vb = s_acc_time(vb1, param2.vb_, param2.a_, param2.j_);

                // mode 1
                if (l2 > (param2.T_ - T_va_to_vb) * std::max(vb1, param2.vb_) + T_va_to_vb * (vb1 + param2.vb_) / 2
                    || l2 < (param2.T_ - T_va_to_vb) * std::min(vb1, param2.vb_) + T_va_to_vb * (vb1 + param2.vb_) / 2) {
                    // mode 0 
                    param2.mode_ = 0;
                    param2.va_ = vb1;
                    auto vc2 = newton_raphson_binary_search([&param2, l2](double vc)->double {
                        double Ta = s_acc_time(param2.va_, vc, param2.a_, param2.j_);
                        double Tb = s_acc_time(param2.vb_, vc, param2.a_, param2.j_);
                        return Ta * (param2.va_ + vc) / 2 + Tb * (param2.vb_ + vc) / 2 + (param2.T_ - Ta - Tb) * vc - l2;
                        }, std::max(vb1, param2.vb_), param2.vc_);
                    param2.vc_ = vc2;
                    param2.Ta_ = s_acc_time(param2.va_, vc2, param2.a_, param2.j_);
                    param2.Tb_ = s_acc_time(param2.vb_, vc2, param2.a_, param2.j_);

                }
                else {
                    // mode 1
                    param2.mode_ = 1;
                    param2.va_ = vb1;

                    double Tc = s_acc_time(param2.va_, param2.vb_, param2.a_, param2.j_);
                    double va = vb1;
                    double v_avg = (param2.T_ - Tc) > 1e-7 ? (l2 - Tc * (param2.va_ + param2.vb_) / 2) / (param2.T_ - Tc) : 0.0;
                    param2.Ta_ = (param2.va_ - param2.vb_) > 1e-10 ? std::abs((v_avg - param2.vb_) / (param2.va_ - param2.vb_)) * (param2.T_ - Tc) : (param2.T_ - Tc) / 2;
                    param2.Tb_ = param2.T_ - param2.Ta_;
                    param2.vc_ = v_avg;
                }






                auto vc2 = newton_raphson_binary_search([&param2, l2](double vc)->double {
                    double Ta = s_acc_time(param2.va_, vc, param2.a_, param2.j_);
                    double Tb = s_acc_time(param2.vb_, vc, param2.a_, param2.j_);
                    return Ta * (param2.va_ + vc) / 2 + Tb * (param2.vb_ + vc) / 2 + aris::dynamic::s_sgn2(vc - param2.vb_) * (param2.T_ - Ta - Tb) * vc - l2;
                    }, vb1, param2.vc_);

            }
            aris::plan::s_scurve_at(param1, param1.T_ + param1.t0_, &p_out);
            p_out = p_out - param1.pa_;
            if (std::abs(p_out - l1) > 1e-9) {
                std::cout << "error" << __LINE__ << std::setprecision(15) << std::endl;
                std::cout << "l :" << l1 << std::endl;
                std::cout << "p_out:" << p_out << std::endl;
                std::cout << "Ta:" << param1.Ta_ << std::endl;
                std::cout << "Tb:" << param1.Tb_ << std::endl;
                std::cout << "T :" << param1.T_ << std::endl;
                std::cout << "va:" << param1.va_ << std::endl;
                std::cout << "vb:" << param1.vb_ << std::endl;
                std::cout << "vc:" << param1.vc_ << std::endl;
                std::cout << "md:" << param1.mode_ << std::endl;
                std::cout << "-----------------------------------------" << std::endl;
                std::cout << "Ta:" << p1.Ta_ << std::endl;
                std::cout << "Tb:" << p1.Tb_ << std::endl;
                std::cout << "T :" << p1.T_ << std::endl;
                std::cout << "va:" << p1.va_ << std::endl;
                std::cout << "vb:" << p1.vb_ << std::endl;
                std::cout << "vc:" << p1.vc_ << std::endl;
                std::cout << "md:" << p1.mode_ << std::endl;
                std::cout << "----------------------------------------------------" << std::endl;
            }

            aris::plan::s_scurve_at(param2, param2.t0_, &p_out);
            if (std::abs(p_out - param2.pa_) > 1e-10) {
                aris::plan::s_scurve_at(param2, param2.t0_, &p_out);

                // mode 1
                param2.mode_ = 1;
                param2.va_ = vb1;

                double Tc = s_acc_time(param2.va_, param2.vb_, param2.a_, param2.j_);
                double va = vb1;
                double v_avg = (param2.T_ - Tc) > 1e-7 ? (l2 - Tc * (param2.va_ + param2.vb_) / 2) / (param2.T_ - Tc) : (param2.va_ + param2.vb_) / 2;
                param2.Ta_ = (param2.va_ - param2.vb_) > 1e-10 ? std::abs((v_avg - param2.vb_) / (param2.va_ - param2.vb_)) * (param2.T_ - Tc) : (param2.T_ - Tc) / 2;
                param2.Tb_ = param2.T_ - param2.Ta_;
                param2.vc_ = v_avg;

                std::cout << "error" << __LINE__ << std::endl;
            }

            aris::plan::s_scurve_at(param2, param2.t0_, &p_out);
            p_out = (param2.T_ - param2.Ta_ - param2.Tb_) * (param2.va_ + param2.vb_) / 2
                + param2.va_ * param2.Ta_ + param2.Tb_ * param2.vb_;
            if (param2.mode_ == 1 && std::abs(p_out - l2) > 1e-10) {
                aris::plan::s_scurve_at(param2, param2.t0_, &p_out);

                std::cout << std::setprecision(15);

                std::cout << param2.Ta_ << std::endl;
                std::cout << param2.Tb_ << std::endl;
                std::cout << param2.T_ << std::endl;
                std::cout << param2.va_ << std::endl;
                std::cout << param2.vb_ << std::endl;
                std::cout << param2.vc_ << std::endl;
                std::cout << l2 << std::endl;
                std::cout << param2.mode_ << std::endl;

                // mode 1
                param2.mode_ = 1;
                param2.va_ = vb1;

                double Tc = s_acc_time(param2.va_, param2.vb_, param2.a_, param2.j_);
                double va = vb1;
                double v_avg = (param2.T_ - Tc) > 1e-7 ? (l2 - Tc * (param2.va_ + param2.vb_) / 2) / (param2.T_ - Tc) : (param2.va_ + param2.vb_) / 2;
                param2.Ta_ = (param2.va_ - param2.vb_) > 1e-10 
                    ? std::abs((v_avg - param2.vb_) / (param2.va_ - param2.vb_)) * (param2.T_ - Tc) 
                    : (param2.T_ - Tc) / 2;
                param2.Tb_ = param2.T_ - param2.Ta_;
                param2.vc_ = v_avg;

                std::cout << "error" << __LINE__ << std::endl;
            }

            aris::plan::s_scurve_at(param1, param1.t0_, &p_out);
            if (std::abs(p_out - param1.pa_) > 1e-10) {
                std::cout << "error" << __LINE__ << std::endl;
            }
#endif
        }

        

    }


    // 以最大时间测试是否可能达到终止条件，即：
    // cond A: 所有末端速度均减为0
    // cond B: 达到最后一个节点，且结束速度满足要求
    auto test_curve_slow(std::list<SCurveNode>::iterator begin, std::list<SCurveNode>::iterator end, double T_min_set)->bool {
        // 达到最后一个节点 //
        if (begin == std::prev(end))
            return true;

        std::vector<double> Tmaxs(begin->params_.size()), Tmins(begin->params_.size());
        for (aris::Size i = 0; i < begin->params_.size(); ++i) {
            // 计算 vb
            s_compute_scurve_node(begin->params_[i]);

            // 更新 vb
            std::next(begin)->params_[i].va_ = begin->params_[i].vb_;

            // 计算 Tmax Tmin
            std::tie(Tmaxs[i], Tmins[i]) = s_compute_scurve_Tmax_Tmin(std::next(begin)->params_[i], T_min_set);
        }

        double Tmin_all = *std::max_element(Tmins.begin(), Tmins.end());
        double Tmax_all = *std::min_element(Tmaxs.begin(), Tmaxs.end());

        if (Tmax_all == std::numeric_limits<double>::infinity()) {
            return true;
        }
        else if (Tmax_all < 0 || Tmax_all < Tmin_all) {
            return false;
        }
        else {
            for (auto& param : std::next(begin)->params_) {
                param.T_ = Tmax_all;
            }
            return test_curve_slow(std::next(begin), end, T_min_set);
        }
    }

    // 循环计算每个节点：
    auto ARIS_API s_compute_scurve(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter, double T_min)->int {
        // 设置正确的 pa, 并检查 vc, a, j 等参数的合理性
        for (auto iter = std::next(begin_iter); iter != end_iter; ++iter) {
            // 设置正确的 pa
            for (Size i = 0; i < iter->params_.size(); ++i) {
                iter->params_[i].pa_ = std::prev(iter)->params_[i].pb_;
                
                if (iter->params_[i].pa_ > iter->params_[i].pb_)
                    THROW_FILE_LINE("POS SET NOT CORRECT");

                if (iter->params_[i].pb_ - iter->params_[i].pa_ >= 100 * std::numeric_limits<double>::epsilon()){
                    if(iter->params_[i].vc_max_ < 1e-10)
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

        // 设置最后的 vb_max 等于 0
        for (Size i = 0; i < std::prev(end_iter)->params_.size(); ++i) {
            std::prev(end_iter)->params_[i].vb_max_ = 0.0;
        }
        
        // 开始循环计算
        for (auto iter = begin_iter; iter != end_iter; ++iter) {
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
            static int count_ = 0;
            //std::cout << "trajectory count:" << count_ << std::endl;
            count_++;
#endif

            // STEP 1 : 计算全部末端的最大最小时间
            std::vector<double> Tmaxs(iter->params_.size()), Tmins(iter->params_.size());
            for (Size i = 0; i < iter->params_.size(); ++i) {
                // 设置正确的 pa 和 va
                if (iter != begin_iter) {
                    iter->params_[i].va_ = std::prev(iter)->params_[i].vb_;
                }
                
                // 计算 Tmax Tmin
                std::tie(Tmaxs[i], Tmins[i]) = s_compute_scurve_Tmax_Tmin(iter->params_[i], T_min);
            }
            
            double Tmin_all = *std::max_element(Tmins.begin(), Tmins.end());
            double Tmax_all = *std::min_element(Tmaxs.begin(), Tmaxs.end());

            // 若起始速度过大，有可能无法规划成功 //
            if (Tmin_all > Tmax_all)
                return -1;

            // STEP 2 : 基于 T_below 求得 T_upper 上限
            double T_upper = Tmax_all;
            double T_below = std::max(Tmin_all, T_min);

            if (T_upper == std::numeric_limits<double>::infinity()) {
                T_upper = std::max(T_below, 1.0) * 2.0; // in case T_below == 0.0
                for (auto& p : iter->params_)
                    p.T_ = T_upper;

                while (!test_curve_slow(iter, end_iter, T_min)) {
                    T_below = T_upper;
                    T_upper = 2.0 * T_upper;

                    for (auto& p : iter->params_)
                        p.T_ = T_upper;
                }
            }

            // STEP 3 : 二分法求解最优的可行时间
            double diff = std::abs(T_upper - T_below);
            double diff_last = 10 * diff;

            while (diff < diff_last) {
                diff_last = diff;
                double T_next = (T_upper + T_below) / 2;
#ifdef DEBUG_ARIS_PLAN_TRAJECTORY
                //std::cout << "  Tnext:" << T_next << std::endl;              
#endif
                for (auto& p : iter->params_)
                    p.T_ = T_next;

                if (test_curve_slow(iter, end_iter, T_min)) {
                    T_upper = T_next;
                } else {
                    T_below = T_next;
                }

                diff = std::abs(T_upper - T_below);
            }

            // STEP 4 : 设置可行时间并计算每段 s 曲线
            for (auto& p : iter->params_) {
                p.T_ = T_upper;
                s_compute_scurve_node(p);

                if (iter != begin_iter) {
                    LargeNum t0 = std::prev(iter)->params_[0].t0_ + std::prev(iter)->params_[0].T_;
                    for (auto& p : iter->params_) {
                        p.t0_ = t0;
                        //p.t0_count_ = std::prev(iter)->params_[0].t0_count_ + std::lround((t0 - std::fmod(t0, 1000.0))/1000.0);
                    }
                }
                
            }
        }

        return 0;
    }



    // 平滑优化scurve
    auto ARIS_API s_optimize_scurve_adjacent_nodes(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter, double T_min)->int {
        // 针对特定的 CASE 来优化
        for (auto iter = begin_iter; iter != end_iter && iter != std::prev(end_iter); ++iter) {
            // 设置正确的 vb_max
            for (Size i = 0; i < iter->params_.size(); ++i) {
                s_optimize_ajacent_nodes(iter->params_[i], std::next(iter)->params_[i]);
            }
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
