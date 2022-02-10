#define ARIS_DEBUG

#include "test_dynamic_matrix.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>

using namespace aris::dynamic;

const double error = 1e-10;

//#define ARIS_DEBUG_DYNAMIC_SVD

namespace aris::dynamic {
	//   
	/*
	template<typename AType, typename UType, typename SType, typename VType>
	auto inline s_svd(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *S, SType s_t, double *V, VType v_t, double zero_check = 1e-10)noexcept->void {
		////////////////////////////////////////////// PART 1: householder变换 /////////////////////////////////
		// 对A做两次 householder 变换：
		// 其中 [ r ] 是第一次的 householder 向量
		// 此外 [ l ] 是第二次的 householder 向量
		//
		// CASE 1: 当 m = n 时
		// 
		// 此时A变为：
		// [ *  r1 r1 ...  r1   r1   r1  ]
		// | *  *  r2 ...  r2   r2   r2  |
		// | l1 *  *  ...  r3   r3   r3  |
		// | l1 l2 *  ...           ..   |
		// | ..       ...  *   rn-2 rn-2 |
		// | .. ..    ...  *    *   rn-1 |
		// [ l1 l2    ... ln-2  *    *   ]
		//
		// 以上共计 n-1 次变换1，n-2次变换2
		//
		// 最终将其变成：
		// 
		// [ *  *  *  ... *    ]
		// | *  *  *  ... 0    |
		// | l1 r1 r1 ... r1   |
		// | .. l2 r2 ... r2   |
		// | .. ..    ...      |
		// [ l1 l2    ... rn-2 ]
		// 同时还需额外一个变量存储rn-1
		//
		// CASE 2: 当 m = n + 1 时
		// 
		// 此时A变为：
		// [ *  r1 r1 ...  r1   r1   r1  ]
		// | *  *  r2 ...  r2   r2   r2  |
		// | l1 *  *  ...  r3   r3   r3  |
		// | l1 l2 *  ...           ..   |
		// | ..       ...  *   rn-2 rn-2 |
		// | .. ..    ...  *    *   rn-1 |
		// | l1 l2    ... ln-2  *    *   |
		// [ l1 l2    ... ln-2 ln-1  *   ]
		//
		// 以上共计 n-1 次变换1，n-1次变换2
		//
		// 最终将其变成：
		// 
		// [ *  *  *  ... *    ]
		// | *  *  *  ... *    |
		// | l1 r1 r1 ... r1   |
		// | .. l2 r2 ... r2   |
		// | .. ..    ...      |
		// [ l1 l2    ... rn-1 ]
		//
		// CASE 3: 当 m > n + 1 时
		// 
		// 此时A变为：
		// [ *  r1 r1 ...  r1   r1   r1  ]
		// | *  *  r2 ...  r2   r2   r2  |
		// | l1 *  *  ...  r3   r3   r3  |
		// | l1 l2 *  ...           ..   |
		// | ..       ...  *   rn-2 rn-2 |
		// | .. ..    ...  *    *   rn-1 |
		// | l1 l2    ... ln-2  *    *   |
		// | l1 l2    ... ln-2 ln-1  *   |
		// [ l1 l2    ... ln-2 ln-1  ln  ]
		//
		// 以上共计 n-1 次变换1，n次变换2
		//
		// 最终将其变成：
		// 
		// [ *  *  *  ... *    ]
		// | *  *  *  ... *    |
		// | l1 r1 r1 ... r1   |
		// | .. l2 r2 ... r2   |
		// | .. ..    ...      |
		// | l1 l2    ... rn-1 |
		// [ l1 l2    ... ln   ]
		
		////////////////////////////////////////////// PART 2: devide and conquer /////////////////////////////////
		// 迭代 
		// [U S V] = dvc(A)
		// S&A: n+1 x n   
		// U  : n+1 x n+1
		// V  :   n x n
		// S 和 A 位于同一片内存
		//
		// A 为 双对角矩阵：
		// [ *         ]
		// | * *       |
		// |   * *     |
		// |     * *   |
		// |       * * |
		// [         * ]
		//
		// 但是在内存中储存为，记做：
		// [ * *  ... * ]
		// | * *  ... * |            
		// |            |
		// [            ]
		// 
		//  STEP 1：递归调用
		//  h = n/2, n为偶数；或 (n-1)/2, n为奇数
		// 	[U1 S1 V1] = dvc(A(   1:h+1 , 1:h   ));
		//  [U2 S2 V2] = dvc(A( h+2:n+1 , h+2:n ));
		//
		//  各矩阵维度：  U1:h+1 x h+1    S1:h+1 x h        V1:    h x h
		//                U2:n-h x n-h    S2:n-h x n-h-1    V2:n-h-1 x n-h-1
		// 
		//  【【注意】】：以上求值顺序不可改变，因后续计算中，需要压缩 V 来省内存空间，因此U2，V2的计算会覆盖V1的最后一行。
		//  【【注意】】：所有返回的V，为了存储紧凑，均存成n-1 x n 维，因为vi中的首个元素均为-1
		// 
		//  调用前的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ EMPTY ]
		//	--------------------------------------------
		//	S：
		//	      h   1   n-h-1 
		//	2   [ A1  ek  A2   ]
		//	n-1 [ EMPTY....... ]
		//	--------------------------------------------
		//	V:
		//	[ empty ]
		//	--------------------------------------------
		// 
		// 
		//  调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ EMPTY ]
		//	--------------------------------------------
		//	S：
		//  1  [ d1       d2 ]
		//  2  | ...empty... |
		// n+1 [ ...empty... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)，此时用额外的3维向量存储 q1 的突出部
		//           1      h   h+1    h+2    n
		//                      q1
		//                      q1
		//      1  [ u1 ... u1  q1     v1 ... v1 ]
		//         | .  ... .   .      v1 ... v1 |
		//         | u1 ... u1  q1     v1 ... v1 |
		//      h  | u1  ...u1  u2     u2 ... q2 |
		//     h+1 | u1 ... u1  u2     u2 ... q2 |
		//     h+2 | v2 ... v2  .      .  ... .  |
		//         | .  ... .   .      .  ... .  |
		//     n-1 | v2 ... v2  u2     .  ... q2 |
		//      n  [ ......   empty  ........    ]
		// 
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h      h+1    h+2    n-1 n
		//      1  [ u1 ... u1   u1     v1     v1 ... v1  v1]
		//         | .  ... .    .      .      .  ... .   . |
		//     h-1 | .  ... .    u1&q1  v1     v1 ... v1  v1|
		//      h  | .  ... .    u1&q1  u2     u2 ... u2  q2|
		//     h+1 | u1 ... u1   u1&q1  u2     u2 ... u2  q2|
		//     h+2 | v2 ... v2   q1     u2     .  ... .   . |
		//         | .  ... .    .      .      .  ... .   . |
		//     n-1 [ v2 ... v2   q1     u2     u2 ... u2  q2]
		//      n  [ .................empty.................]
		//
		// STEP 2：构造  d  z  mu  p
		// 
		// 其中：
		// d 和 z 构成了 M：
		// [ z1            ]
		// | z2  d2        |
		// | ...    ...    |
		// [ zn         dn ]
		//
		// 以上内存分别存储于不同的地方，因此调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ EMPTY ]
		//	--------------------------------------------
		//	S：
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  p  ... |
		// n+1 [ ...empty... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)，此时用额外的3维向量存储 q1 的突出部
		// 
		//           1      h   h+1    h+2    n
		//                      q1
		//                      q1
		//      1  [ u1 ... u1  q1     v1 ... v1 ]
		//         | .  ... .   .      v1 ... v1 |
		//         | u1 ... u1  q1     v1 ... v1 |
		//      h  | u1  ...u1  u2     u2 ... q2 |
		//     h+1 | u1 ... u1  u2     u2 ... q2 |
		//     h+2 | v2 ... v2  .      .  ... .  |
		//         | .  ... .   .      .  ... .  |
		//     n-1 | v2 ... v2  u2     .  ... q2 |
		//      n  [ .............mu.............]
		// 
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h      h+1    h+2    n-1 n
		//      1  [ u1 ... u1   u1     v1     v1 ... v1  v1]
		//         | .  ... .    .      .      .  ... .   . |
		//     h-1 | .  ... .    u1&q1  v1     v1 ... v1  v1|
		//      h  | .  ... .    u1&q1  u2     u2 ... u2  q2|
		//     h+1 | u1 ... u1   u1&q1  u2     u2 ... u2  q2|
		//     h+2 | v2 ... v2   q1     u2     .  ... .   . |
		//         | .  ... .    .      .      .  ... .   . |
		//     n-1 [ v2 ... v2   q1     u2     u2 ... u2  q2]
		//      n  [ ...................mu..................]
		// 
		// STEP 3：构造 U
		// 
		// 在该步中，ui存于S的第4行处(因 n+1>=4)
		// 
		//  调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ full of U ]
		//	--------------------------------------------
		//	S：
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  p  ... |
		//  4  | ...  ui ... |
		// n+1 [ ...empty... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)，此时用额外的3维向量存储 q1 的突出部
		// 
		//           1      h   h+1  h+2    n
		//      1  [ .....empty..... v1 ... v1 ]
		//         | .....empty..... v1 ... v1 |
		//         | .....empty..... v1 ... v1 |
		//      h  | .....empty.........empty..|
		//     h+1 | .....empty.........empty..|
		//     h+2 | v2 ... v2 .........empty..|
		//         | .  ... .  .........empty..|
		//     n-1 | v2 ... v2 .........empty..|
		//      n  [ ..........mu..............]
		// 
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h   h+1 h+2    n-1 n
		//      1  [ .....empty..... v1  v1 ... v1  v1]
		//         | .....empty..... .   .  ... .   . |
		//     h-1 | .....empty..... v1  v1 ... v1  v1|
		//      h  | .....empty..........empty........|
		//     h+1 | .....empty..........empty........|
		//     h+2 | v2 ... v2 ..........empty........|
		//         | .  ... .  ..........empty........|
		//     n-1 | v2 ... v2 ..........empty........|
		//      n  [ ...............mu................]
		// 
		// 
		// 
		// STEP 4：构造 V
		// 
		// 在该步中，vi可能存于p处，或S的最后一行处
		// 
		// vi位置：
		//	--------------------------------------------
		//	S：
		//    n >= 5 时：
		//       
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  p  ... |
		// 4.. | V1       V2 |
		// n+1 [ ...  vi ... ]
		//    n < 5 时：(p存于其他地方)
		//       
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  vi ... |
		// 4.. | V1       V2 |
		//	--------------------------------------------
		// 
		// 
		//  调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ full of U ]
		//	--------------------------------------------
		//	S：
		//       
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  p  ... |
		//  4  | ...  ui ... |
		// n+1 [ ...empty... ]
		//	--------------------------------------------
		//
		//	V：
		//   1:n-1 [ compressed V ]
		//      n  [ ....mu.......]
		// 
		// STEP 5：构造 S
		//
		//  调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ full of U ]
		//	--------------------------------------------
		//	S：
		//       
		//  1     [ ...  d  ... ]
		//  2:n+1 [ ...empty... ]
		//	--------------------------------------------
		//
		//	V：
		//   1:n-1 [ compressed V ]
		//      n  [ ...empty.....]

		////////////////////////////////////////////// PART 3: 首次调用 /////////////////////////////////
		//  首次经过dvc调用后，整体的内存布局如下：
		//  --------------------------------------------
		//	U：
		//	1:n-1 [ EMPTY ]
		//  n     [...p...]
		//  n+1:m [ EMPTY ]
		//	--------------------------------------------
		//	S：
		//	1   [ ......d....... ]
		//	2   | ......z....... |
		//	n-1 [ househoders... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)，此时用额外的3维向量存储 q1 的突出部
		//           1      h   h+1    h+2    n
		//                      q1
		//                      q1
		//      1  [ u1 ... u1  q1     v1 ... v1 ]
		//         | .  ... .   .      v1 ... v1 |
		//         | u1 ... u1  q1     v1 ... v1 |
		//      h  | u1  ...u1  u2     u2 ... q2 |
		//     h+1 | u1 ... u1  u2     u2 ... q2 |
		//     h+2 | v2 ... v2  .      .  ... .  |
		//         | .  ... .   .      .  ... .  |
		//     n-1 | v2 ... v2  u2     .  ... q2 |
		//      n  [ .........mu.................]
		// 
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h      h+1    h+2    n-1 n
		//      1  [ u1 ... u1   u1     v1     v1 ... v1  v1 ]
		//         | .  ... .    .      .      .  ... .   .  |
		//     h-1 | .  ... .    u1&q1  v1     v1 ... v1  v1 |
		//      h  | .  ... .    u1&q1  u2     u2 ... u2  q2 |
		//     h+1 | u1 ... u1   u1&q1  u2     u2 ... u2  q2 |
		//     h+2 | v2 ... v2   q1     u2     .  ... .   .  |
		//         | .  ... .    .      .      .  ... .   .  |
		//     n-1 | v2 ... v2   q1     u2     u2 ... u2  q2 |
		//      n  [ ..................mu....................]
		//
		//  STEP 1:计算U
		//  若 n > 6 时，先计算U1所产生的 U（不做householder变换），此后将此时内存变化如下：
		//  --------------------------------------------
		//	U：
		//	1    [ u ... u ]
		//  ...  |         |
		//  h+1  | u ... u |
		//  ...  | .empty. |
		//  n    | .empty. |
		//  ...  | .empty. |
		//  m    [ .empty. ]
		//	--------------------------------------------
		//	S：
		//	1   [ ......d....... ]
		//	2   | ......z....... |
		//	n-1 [ househoders... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)
		//           1      h   h+1    h+2    n
		//      1  [ ...........ui.............. ]
		//      2  | ...........p............... |
		//         | ....EMPTY....     v1 ... v1 |
		//      h  | v1 ... v1  u2     u2 ... q2 |
		//     h+1 | v1 ... v1  u2     u2 ... q2 |
		//     h+2 | v2 ... v2  .      .  ... .  |
		//         | .  ... .   .      .  ... .  |
		//     n-1 | v2 ... v2  u2     .  ... q2 |
		//      n  [ .........mu.................]
		// 
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h   h+1    h+2    n-1 n
		//      1  [ ...........ui....................... ]
		//      2  | ...........p........................ |
		//         | ....EMPTY....   v1     v1 ... v1  v1 |
		//     h-1 | ....EMPTY....   v1     v1 ... v1  v1 |
		//      h  | v1 ... v1   v1  u2     u2 ... u2  q2 |
		//     h+1 | v1 ... v1   v1  u2     u2 ... u2  q2 |
		//     h+2 | v2 ... v2       u2     .  ... .   .  |
		//         | .  ... .        .      .  ... .   .  |
		//     n-1 | v2 ... v2       u2     u2 ... u2  q2 |
		//      n  [ ..................mu.................]
		//
		//  再计算U2产生的矩阵，并进行反householder变换，结果如下：
		//  
		//  --------------------------------------------
		//	U：
		//	    [ full of U ]
		//	--------------------------------------------
		//	S：
		//	1   [ .......d....... ]
		//	2   | .......z....... |
		//	3   | *  r  ......... |
		//	4   | *  *  r  ...... |
		//	... | .......p....... |
		//	n   | ......mu....... |    // 右下角用r_corner 来临时保存，为3 x 4 的矩阵
		//  ... [ ............... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  
		//           1      h   h+1    h+2    n
		//      1  [ .....EMPTY.....   v1 ... v1 ]
		//         | .....EMPTY.....   v1 ... v1 |
		//         | .....EMPTY.....   v1 ... v1 |
		//      h  | .....EMPTY..........EMPTY.. |
		//     h+1 | .....EMPTY..........EMPTY.. |
		//     h+2 | v2 ... v2  .........EMPTY.. |
		//         | .  ... .   .........EMPTY.. |
		//     n-1 | v2 ... v2  .........EMPTY.. |
		//      n  [ .....EMPTY..........EMPTY.. ]
		// 
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h      h+1    h+2    n-1 n
		//      1  [ .....EMPTY.....    v1     v1 ... v1  v1 ]
		//         | .....EMPTY.....    .      .  ... .   .  |
		//     h-1 | .....EMPTY.....    v1     v1 ... v1  v1 |
		//      h  | .....EMPTY..............EMPTY.......... |
		//     h+1 | .....EMPTY..............EMPTY.......... |
		//     h+2 | v2 ... v2   ............EMPTY.......... |
		//         | .  ... .    ............EMPTY.......... |
		//     n-1 | v2 ... v2   ............EMPTY.......... |
		//      n  [ .....EMPTY..............EMPTY.......... ]
		// 
		//
		//  STEP 2:计算V
		//  若 n > 6 时，先计算V1所产生的 V（不做householder变换），此后将此时内存变化如下：
		//  --------------------------------------------
		//	U：
		//	    [ full of U ]
		//	--------------------------------------------
		//	S：
		//          1  2  3 ... h
		//	1     [ .......d.......... ]
		//	2     | .......z.......... |
		//	3     | *  r  ............ |
		//	4     | *  *  r  ......... |
		//  ...   |                    |
		//  n-1-h | v1 ........ v1 r   |    // 右上方需要保存起点为 n-1-h, h-3 的 3x3 的矩阵
		//  ...   |                    |
		//  n-2   | v1 ........ v1 r   |
		//	n-1   | .......p.......... |
		//	n     | ......mu.......... |    // 右下角用r_corner 来临时保存，为3 x 4 的矩阵
		//  ...   [ .................. ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  
		//           1    n-h-1              n
		//      1  [ .....EMPTY..........EMPTY.. ]
		//         | .....EMPTY..........EMPTY.. |
		//     h+1 | .....EMPTY..........EMPTY.. |
		//     h+2 | v2 ... v2  .........EMPTY.. |
		//         | .  ... .   .........EMPTY.. |
		//     n-1 | v2 ... v2  .........EMPTY.. |
		//      n  [ .....EMPTY..........EMPTY.. ]
		//
		//
		//

		// check size //
		if (m == 0 || n == 0) return;

		// check m > n
		if (n > m) {
			s_svd(n, m, A, T(a_t), V, v_t, S, T(s_t), U, u_t);
			return;
		}

		// n == 1 //
		if (n == 1) {
			s_mc(m, n, A, a_t, S, s_t);

			if (m == 1) {
				U[at(0, 0, u_t)] = 1;
				V[at(0, 0, v_t)] = s_sgn2(S[at(0, 0, s_t)]);
				S[at(0, 0, s_t)] = s_sgn2(S[at(0, 0, s_t)]) * S[at(0, 0, s_t)];
			}
			else {
				// householder 变换 //
				double rho = -s_sgn2(S[at(0, 0, s_t)]) * std::sqrt(s_vv(m, S + at(0, 0, s_t), s_t, S + at(0, 0, s_t), s_t));
				s_nm(m - 1, 1, std::abs(rho) > zero_check ? 1.0 / (S[at(0, 0, s_t)] - rho) : 0.0, S + at(1, 0, s_t), s_t);
				
				// U //
				s_eye(m, U, u_t);
				s_householder_u_q_dot(m, n, m, S + at(0, 0, s_t), s_t, U + at(0, 0, u_t), u_t, U + at(0, 0, u_t), u_t);

				// V //
				V[at(0, 0, v_t)] = s_sgn2(rho);

				// S //
				s_fill(m, n, 0.0, S, s_t);
				S[at(0, 0, s_t)] = s_sgn2(rho) * rho;
			}
			return;
		}
		
		// n == 2 //
		if (n == 2) {
			s_mc(m, n, A, a_t, S, s_t);
			
			double tau[2];
			s_householder_ut(m, n, S, s_t, S, s_t, tau, 1);

			auto a = S[at(0, 0, s_t)];
			auto b = S[at(0, 1, s_t)];
			auto d = S[at(1, 1, s_t)];

			auto theta3 = 0.5 * std::atan2(2 * b * d, a * a + b * b - d * d);
			auto c3 = std::cos(theta3);
			auto s3 = std::sin(theta3);

			auto S1 = a * a + b * b + d * d;
			auto S2 = std::sqrt((a * a + b * b - d * d) * (a * a + b * b - d * d) + 4 * b * b * d * d);

			auto phi = 0.5 * std::atan2(2 * a * b, a * a - b * b - d * d);
			auto c_p = std::cos(phi);
			auto s_p = std::sin(phi);
			auto s11 = (a * c3) * c_p + (b * c3 + d * s3) * s_p;
			auto s22 = (a * s3) * s_p + (-b * s3 + d * c3) * c_p;

			// U //
			s_eye(m, U, u_t);
			U[at(0, 0, u_t)] = s_sgn2(s11) * (c3);
			U[at(0, 1, u_t)] = s_sgn2(s22) * (-s3);
			U[at(1, 0, u_t)] = s_sgn2(s11) * (s3);
			U[at(1, 1, u_t)] = s_sgn2(s22) * (c3);

			// 对U进行反 householder 变换 //
			s_householder_ut_q_dot(m, n, m, S + at(0, 0, s_t), s_t, tau, 1, U + at(0, 0, u_t), u_t, U + at(0, 0, u_t), u_t);

			// V //
			V[at(0, 0, v_t)] = c_p;
			V[at(0, 1, v_t)] = -s_p;
			V[at(1, 0, v_t)] = s_p;
			V[at(1, 1, v_t)] = c_p;

			// S //
			s_fill(m, n, 0.0, S, s_t);
			S[at(0, 0, s_t)] = std::sqrt((S1 + S2) / 2);
			S[at(1, 1, s_t)] = std::sqrt((S1 - S2) / 2);

			return;
		}

		//////////////////////////////////////////////////////////// PART 1: householder 变换 /////////////////////////////////
		s_mc(m, n, A, a_t, S, s_t);
		for (Size i = 0; i < n; ++i) {
			// compute householder vector 1 //
			if (i < n - 1){
				double rho = -s_sgn2(S[at(i, i, s_t)]) * std::sqrt(s_vv(n - i, S + at(i, i, s_t), T(s_t), S + at(i, i, s_t), T(s_t)));
				double tau_reciprocal = std::abs(rho) > zero_check ? S[at(i, i, s_t)] / rho - 1.0 : -2.0;   // tau 的倒数，tau和论文中定义一致
				s_nm(1, n - 1 - i, std::abs(rho) > zero_check ? 1.0 / (S[at(i, i, s_t)] - rho) : 0.0, S + at(i, i + 1, s_t), s_t);
				
				// update matrix 1 //
				for (Size j(i); ++j < m;) {
					double k = tau_reciprocal * (s_vv(n - i - 1, S + at(i, i + 1, s_t), T(s_t), S + at(j, i + 1, s_t), T(s_t)) + S[at(j, i, s_t)]);
					S[at(j, i, s_t)] += k;
					s_ma(1, n - i - 1, k, S + at(i, i + 1, s_t), s_t, S + at(j, i + 1, s_t), s_t);
				}
				S[at(i, i, s_t)] = rho;
			}

			// compute householder vector 2, CASE 1 2 3 //
			if ((n > 1 && i < n - 2) || (m == n + 1 && i < n - 1) || (m > n + 1 && i < n)) {
				double rho = -s_sgn2(S[at(i + 1, i, s_t)]) * std::sqrt(s_vv(m - i - 1, S + at(i + 1, i, s_t), s_t, S + at(i + 1, i, s_t), s_t));
				double tau_reciprocal = std::abs(rho) > zero_check ? S[at(i + 1, i, s_t)] / rho - 1.0 : -2.0;   // tau 的倒数，tau和论文中定义一致
				s_nm(m - 2 - i, 1, std::abs(rho) > zero_check ? 1.0 / (S[at(i + 1, i, s_t)] - rho) : 0.0, S + at(i + 2, i, s_t), s_t);

				// update matrix 2 //
				for (Size j(i); ++j < n;) {
					double k = tau_reciprocal * (s_vv(m - i - 2, S + at(i + 2, i, s_t), s_t, S + at(i + 2, j, s_t), s_t) + S[at(i + 1, j, s_t)]);
					S[at(i + 1, j, s_t)] += k;
					s_va(m - i - 2, k, S + at(i + 2, i, s_t), s_t, S + at(i + 2, j, s_t), s_t);
				}
				S[at(i + 1, i, s_t)] = rho;
			}

			// 用 U 来存储 d 和 z //
			U[at(0, i, u_t)] = S[at(i, i, s_t)];
			U[at(1, i, u_t)] = m == n && i == n - 1 ? 0.0 : S[at(i + 1, i, s_t)];
		}

		// reconstruct matrix //
		double householder_vec_n[2]{ 0.0, S[at(n - 2, n - 1, s_t)] };
		for (Size i(1); ++i < n;) {
			s_mc(1, i, S + at(n - 1 - i, n - i, s_t), s_t, S + at(n + 1 - i, n - i, s_t), s_t);
		}

		//////////////////////////////////////////////////////////// PART 2: devide and conquer ///////////////////////////////
#ifdef ARIS_DEBUG_DYNAMIC_SVD
		struct DebugStruct {
			int level;
			int num;
			Size n;
			std::vector<double> U, S, V;
		};
		std::list<DebugStruct> _D_list;
#endif
		struct DvcPreOut {
			Size mn;
			double dn;
			std::array<double, 3> q1_01, q2_01;
			double q2_0, q1_e;
			double r0, c0, s0;
			std::array<double, 3> q01;
			Size mn1, mn2;
		};

		const auto dvc_pre = [
#ifdef ARIS_DEBUG_DYNAMIC_SVD
			&
#endif
		](Size n, double* S, auto s_t, double* V, VType v_t, const auto& dvc, const auto& dvc_pre)->DvcPreOut
		{
#ifdef ARIS_DEBUG_DYNAMIC_SVD
			DebugStruct _D_s;
			_D_s.n = n;
			_D_s.U.resize((n + 1)* (n + 1));
			_D_s.S.resize((n + 1)* (n));
			_D_s.V.resize((n)* (n));

			s_mc(n + 1, n + 1, U, u_t, _D_s.U.data(), n + 1);
			s_mc(n + 1, n,     S, s_t, _D_s.S.data(), n);
			s_mc(n,     n,	   V, v_t, _D_s.V.data(), n);

			if (_D_list.empty()) {
				_D_s.level = 0;
				_D_s.num = 0;
				_D_list.push_back(_D_s);
			}
			else {
				_D_s.num = 0;
				_D_s.level = _D_list.back().level + 1;
				_D_list.push_back(_D_s);
			}
#endif
			DvcPreOut pre_out = {};

			const Size MAX_N = 9;

			if (n > 2 && n < MAX_N) {
				auto h = n / 2;
				auto U1 = V + at(0, 0, v_t);
				auto u1_t = v_t;
				auto S1 = S + at(0, 0, s_t);
				auto s1_t = s_t;
				auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
				auto v1_t = v_t;
				auto U2 = V + at(h - 1, h, v_t);
				auto u2_t = v_t;
				auto S2 = S + at(0, h + 1, s_t);
				auto s2_t = s_t;
				auto V2 = V + at(h + 1, 0, v_t);
				auto v2_t = v_t;

				auto q1 = V + (h == n - h ? at(h + 1, h - 1, v_t) : at(0, h, v_t));
				auto q2 = U2 + at(0, n - h - 1, u2_t);

				const auto a_h_h = S[at(0, h, s_t)];
				const auto a_hp1_h = S[at(1, h, s_t)];

				auto ret1 = dvc(h, U1, u1_t, q1, u1_t, S1, s1_t, V1, v1_t, dvc, dvc_pre);
				auto q1_e = h < 3 ? ret1.q01[h] : q1[at(h - 3, 0, u1_t)];

				auto ret2 = dvc(n - h - 1, U2, u2_t, q2, u2_t, S2, s2_t, V2, v2_t, dvc, dvc_pre);
				auto q2_0 = ret2.q01[0];

				// # STEP 2：构造 d、z、mu、p  //
				// ## STEP 2.1：计算排序的 d z p  //
				auto r0 = std::sqrt(a_h_h * a_h_h * q1_e * q1_e + a_hp1_h * a_hp1_h * q2_0 * q2_0);
				auto theta = std::atan2(a_hp1_h * q2_0, a_h_h * q1_e);
				auto c0 = std::cos(theta);
				auto s0 = std::sin(theta);

				auto d = S;
				auto d_t = s_t;
				auto z = S + at(1, 0, s_t);
				auto z_t = s_t;
				auto mu = V + at(n - 1, 0, v_t);
				auto mu_t = v_t;
				auto p = S + at(2, 0, s_t);
				auto p_t = s_t;

				// 把S1的内存右移一格
				for (Size i = h; --i < h;) {
					d[at(0, i + 1, d_t)] = d[at(0, i, d_t)];
				}
				d[at(0, 0, d_t)] = 0.0;
				z[at(0, 0, z_t)] = r0;

				s_mc(1, h, a_h_h, U1 + at(h, 0, u1_t), u1_t, z + at(0, 1, z_t), z_t);
				s_mc(1, n - h - 2, a_hp1_h, U2 + at(0, 1, u2_t), u2_t, z + at(0, h + 2, z_t), z_t);
				z[at(0, h + 1, z_t)] = a_hp1_h * U2[at(0, 0, u2_t)];// U2[0,0] 不正确，因为内存重叠

				// 排序生成p
				for (auto i = 0; i < n; ++i)p[at(0, i, p_t)] = i;
				std::sort(RowIterator<decltype(p_t)>(p, p_t), RowIterator<decltype(p_t)>(p, p_t) + n, [&d, &d_t](const auto& left, const auto& right) {
					return d[at(0, static_cast<Size>(left), d_t)] < d[at(0, static_cast<Size>(right), d_t)];
					});

				// d和z两个一起变换
				s_permutate(n, 2, RowIterator<decltype(p_t)>(p, p_t), d, T(d_t));

				// ## STEP 2.2：deflation
				auto max_abs_z = std::abs(*std::max_element(RowIterator<decltype(z_t)>(z, z_t), RowIterator<decltype(z_t)>(z, z_t) + n, [](const auto& l, const auto& r) {return std::abs(l) < std::abs(r); }));
				auto max_d = d[at(0, n - 1, d_t)];
				constexpr auto consider_zero = std::numeric_limits<double>::min();
				const auto epsilon_strict = std::max(consider_zero, std::numeric_limits<double>::epsilon() * max_d);
				const auto epsilon_coarse = std::max(consider_zero, 8.0 * std::numeric_limits<double>::epsilon() * std::max(max_abs_z, max_d));

				auto mn = n;
				for (Size i = 1; i < mn; ) {
					// check zi near zero, deflation type 2
					if (std::abs(z[at(0, i, z_t)]) < epsilon_strict) {
						mu[at(0, mn - 1, mu_t)] = -2;
						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);
						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;
						--mn;

						continue;
					}
					else if (std::abs(d[at(0, i, d_t)]) < epsilon_coarse) {
						mu[at(0, mn - 1, mu_t)] = -1;
						auto z1 = std::sqrt(z[at(0, 0, z_t)] * z[at(0, 0, z_t)] + z[at(0, i, z_t)] * z[at(0, i, z_t)]);
						auto zi = std::atan2(z[at(0, i, z_t)], z[at(0, 0, z_t)]);
						z[at(0, 0, z_t)] = z1;
						z[at(0, i, z_t)] = zi;
						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);
						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;
						--mn;

						continue;
					}
					else if (d[at(0, i, d_t)] - d[at(0, i - 1, d_t)] < epsilon_strict) {
						mu[at(0, mn - 1, mu_t)] = (double)(i - 1);
						auto z_im1 = std::sqrt(z[at(0, i, z_t)] * z[at(0, i, z_t)] + z[at(0, i - 1, z_t)] * z[at(0, i - 1, z_t)]);
						auto z_i = std::atan2(z[at(0, i, z_t)], z[at(0, i - 1, z_t)]);
						z[at(0, i - 1, z_t)] = z_im1;
						z[at(0, i, z_t)] = z_i;

						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);

						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;

						--mn;
					}
					else {
						++i;
					}
				}

				auto dn = 0.0;
				if (mn == 1) {
					mu[at(0, mn - 1, mu_t)] = -2;
					mn = 0;
					d[at(0, 0, d_t)] = z[at(0, 0, z_t)];
					dn = z[at(0, 0, z_t)];
				}
				else {
					if (z[at(0, 0, z_t)] <= epsilon_coarse)
						z[at(0, 0, z_t)] = epsilon_coarse;

					dn = d[at(0, mn - 1, d_t)] + std::sqrt(s_vv(mn, z, T(z_t), z, T(z_t)));
				}

				// ## STEP 2.3：计算奇异值(mu)
				for (Size i{ 0 }, di{ 0 }; i < mn; ++i, di = next_c(di, d_t)) {
					auto left = d[di];
					auto right = i == mn - 1 ? dn : d[next_c(di, d_t)];
					auto mid = 0.5 * (left + right);
					auto v = 0.0;
					for (Size j = 0; j < mn; ++j) {
						v += z[at(0, j, z_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - mid) / (d[at(0, j, d_t)] + mid);
					}

					auto base = v < -1 ? right : left;
					auto lower = v < -1 ? left - right : 0.0;
					auto upper = v < -1 ? 0.0 : right - left;

					while (std::abs(lower - upper) > (std::max(std::abs(lower), std::abs(upper))) * 2 * std::numeric_limits<double>::epsilon()) {
						mid = (lower + upper) / 2;
						auto v = 0.0;
						for (Size j = 0; j < mn; ++j) {
							v += z[at(0, j, z_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mid) / (d[at(0, j, d_t)] + base + mid);
						}
						v < -1 ? lower = mid : upper = mid;
					}
					mu[at(0, i, mu_t)] = (lower + upper) / 2;
				}

				// ## STEP 2.4：重新计算 z
				for (Size i = 0, d_i = 0, z_i = 0; i < mn; ++i, d_i = next_c(d_i, d_t), z_i = next_c(z_i, z_t)) {
					auto base = mu[at(0, mn - 1, mu_t)] < 0.0 ? dn : d[at(0, mn - 1, d_t)];
					auto zi = (base + d[at(0, i, d_t)] + mu[at(0, mn - 1, mu_t)]) * (base - d[at(0, i, d_t)] + mu[at(0, mn - 1, mu_t)]);

					for (Size k = 0, mu_k = 0, d_k = 0, d_kp1 = next_c(d_k, d_t); k < i; ++k, mu_k = next_c(mu_k, mu_t), d_k = d_kp1, d_kp1 = next_c(d_k, d_t)) {
						base = mu[mu_k] < 0 ? d[d_kp1] : d[d_k];
						zi *= (base - d[d_i] + mu[mu_k]) * (base + d[d_i] + mu[mu_k]) / (d[d_k] - d[d_i]) / (d[d_k] + d[d_i]);
					}

					for (Size k = i, mu_k = at(0, k, mu_t), d_k = at(0, k, d_t), d_kp1 = next_c(d_k, d_t); k < mn - 1; ++k, mu_k = next_c(mu_k, mu_t), d_k = d_kp1, d_kp1 = next_c(d_k, d_t)) {
						base = mu[mu_k] < 0 ? d[d_kp1] : d[d_k];
						zi *= (base - d[d_i] + mu[mu_k]) * (base + d[d_i] + mu[mu_k]) / (d[d_kp1] - d[d_i]) / (d[d_kp1] + d[d_i]);
					}

					z[z_i] = s_sgn2(z[z_i]) * std::sqrt(zi);
				}

				pre_out = DvcPreOut{
					mn,
					dn,
					ret1.q01, ret2.q01,
					q2_0, q1_e,
					r0, c0, s0,
					std::array<double,3>{0.0, 0.0, 0.0},
					ret1.mn, ret2.mn,
				};
			}
			else if(n > 2) {
				auto h = n / 2;
				auto U1 = V + at(0, 0, v_t);
				auto u1_t = v_t;
				auto S1 = S + at(0, 0, s_t);
				auto s1_t = s_t;
				auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
				auto v1_t = v_t;
				auto U2 = V + at(h - 1, h, v_t);
				auto u2_t = v_t;
				auto S2 = S + at(0, h + 1, s_t);
				auto s2_t = s_t;
				auto V2 = V + at(h + 1, 0, v_t);
				auto v2_t = v_t;

				auto q1 = V + (h == n - h ? at(h + 1, h - 1, v_t) : at(0, h, v_t));
				auto q2 = U2 + at(0, n - h - 1, u2_t);

				const auto a_h_h = S[at(0, h, s_t)];
				const auto a_hp1_h = S[at(1, h, s_t)];

				auto ret1 = dvc(h, U1, u1_t, q1, u1_t, S1, s1_t, V1, v1_t, dvc, dvc_pre);
				auto q1_e = h < 3 ? ret1.q01[h] : q1[at(h - 3, 0, u1_t)];

				auto ret2 = dvc(n - h - 1, U2, u2_t, q2, u2_t, S2, s2_t, V2, v2_t, dvc, dvc_pre);
				auto q2_0 = ret2.q01[0];

				// # STEP 2：构造 d、z、mu、p  //
				// ## STEP 2.1：计算排序的 d z p  //
				auto r0 = std::sqrt(a_h_h * a_h_h * q1_e * q1_e + a_hp1_h * a_hp1_h * q2_0 * q2_0);
				auto theta = std::atan2(a_hp1_h * q2_0, a_h_h * q1_e);
				auto c0 = std::cos(theta);
				auto s0 = std::sin(theta);

				auto d = S;
				auto d_t = s_t;
				auto z = S + at(1, 0, s_t);
				auto z_t = s_t;
				auto mu = V + at(n - 1, 0, v_t);
				auto mu_t = v_t;
				auto p = S + at(2, 0, s_t);
				auto p_t = s_t;

				// 把S1的内存右移一格
				for (Size i = h; --i < h;) {
					d[at(0, i + 1, d_t)] = d[at(0, i, d_t)];
				}
				d[at(0, 0, d_t)] = 0.0;
				z[at(0, 0, z_t)] = r0;

				s_mc(1, h, a_h_h, U1 + at(h, 0, u1_t), u1_t, z + at(0, 1, z_t), z_t);
				s_mc(1, n - h - 2, a_hp1_h, U2 + at(0, 1, u2_t), u2_t, z + at(0, h + 2, z_t), z_t);
				z[at(0, h + 1, z_t)] = a_hp1_h * U2[at(0, 0, u2_t)];// U2[0,0] 不正确，因为内存重叠

				// 排序生成p
				for (auto i = 0; i < n; ++i)p[at(0, i, p_t)] = i;
				std::sort(RowIterator<decltype(p_t)>(p, p_t), RowIterator<decltype(p_t)>(p, p_t) + n, [&d, &d_t](const auto& left, const auto& right) {
					return d[at(0, static_cast<Size>(left), d_t)] < d[at(0, static_cast<Size>(right), d_t)];
				});

				// d和z两个一起变换
				s_permutate(n, 2, RowIterator<decltype(p_t)>(p, p_t), d, T(d_t));

				// ## STEP 2.2：deflation
				auto max_abs_z = std::abs(*std::max_element(RowIterator<decltype(z_t)>(z, z_t), RowIterator<decltype(z_t)>(z, z_t) + n, [](const auto& l, const auto& r) {return std::abs(l) < std::abs(r); }));
				auto max_d = d[at(0, n - 1, d_t)];
				constexpr auto consider_zero = std::numeric_limits<double>::min();
				const auto epsilon_strict = std::max(consider_zero, std::numeric_limits<double>::epsilon() * max_d);
				const auto epsilon_coarse = std::max(consider_zero, 8.0 * std::numeric_limits<double>::epsilon() * std::max(max_abs_z, max_d));

				auto mn = n;
				for (Size i = 1; i < mn; ) {
					// check zi near zero, deflation type 2
					if (std::abs(z[at(0, i, z_t)]) < epsilon_strict) {
						mu[at(0, mn - 1, mu_t)] = -2;
						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);
						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;
						--mn;

						continue;
					}
					else if (std::abs(d[at(0, i, d_t)]) < epsilon_coarse) {
						mu[at(0, mn - 1, mu_t)] = -1;
						auto z1 = std::sqrt(z[at(0, 0, z_t)] * z[at(0, 0, z_t)] + z[at(0, i, z_t)] * z[at(0, i, z_t)]);
						auto zi = std::atan2(z[at(0, i, z_t)], z[at(0, 0, z_t)]);
						z[at(0, 0, z_t)] = z1;
						z[at(0, i, z_t)] = zi;
						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);
						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;
						--mn;

						continue;
					}
					else if (d[at(0, i, d_t)] - d[at(0, i - 1, d_t)] < epsilon_strict) {
						mu[at(0, mn - 1, mu_t)] = (double)(i - 1);
						auto z_im1 = std::sqrt(z[at(0, i, z_t)] * z[at(0, i, z_t)] + z[at(0, i - 1, z_t)] * z[at(0, i - 1, z_t)]);
						auto z_i = std::atan2(z[at(0, i, z_t)], z[at(0, i - 1, z_t)]);
						z[at(0, i - 1, z_t)] = z_im1;
						z[at(0, i, z_t)] = z_i;

						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);

						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;

						--mn;
					}
					else{
						++i;
					}
				}

				auto dn = 0.0;
				if (mn == 1) {
					mu[at(0, mn - 1, mu_t)] = -2;
					mn = 0;
					d[at(0, 0, d_t)] = z[at(0, 0, z_t)];
					dn = z[at(0, 0, z_t)];
				}
				else {
					if (z[at(0, 0, z_t)] <= epsilon_coarse)
						z[at(0, 0, z_t)] = epsilon_coarse;

					dn = d[at(0, mn - 1, d_t)] + std::sqrt(s_vv(mn, z, T(z_t), z, T(z_t)));
				}

				// ## STEP 2.3：计算奇异值(mu)
				for (Size i{ 0 }, di{ 0 }; i < mn; ++i, di = next_c(di, d_t)) {
					auto left = d[di];
					auto right = i == mn - 1 ? dn : d[next_c(di, d_t)];
					auto mid = 0.5 * (left + right);
					auto v = 0.0;
					for (Size j = 0; j < mn; ++j) {
						v += z[at(0, j, z_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - mid) / (d[at(0, j, d_t)] + mid);
					}

					auto base = v < -1 ? right : left;
					auto lower = v < -1 ? left - right : 0.0;
					auto upper = v < -1 ? 0.0 : right - left;

					while (std::abs(lower - upper) > (std::max(std::abs(lower), std::abs(upper))) * 2 * std::numeric_limits<double>::epsilon()) {
						mid = (lower + upper) / 2;
						auto v = 0.0;
						for (Size j = 0; j < mn; ++j) {
							v += z[at(0, j, z_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mid) / (d[at(0, j, d_t)] + base + mid);
						}
						v < -1 ? lower = mid : upper = mid;
					}
					mu[at(0, i, mu_t)] = (lower + upper) / 2;
				}

				// ## STEP 2.4：重新计算 z
				for (Size i = 0, d_i = 0, z_i = 0; i < mn; ++i, d_i = next_c(d_i, d_t), z_i = next_c(z_i, z_t)) {
					auto base = mu[at(0, mn - 1, mu_t)] < 0.0 ? dn : d[at(0, mn - 1, d_t)];
					auto zi = (base + d[at(0, i, d_t)] + mu[at(0, mn - 1, mu_t)]) * (base - d[at(0, i, d_t)] + mu[at(0, mn - 1, mu_t)]);

					for (Size k = 0, mu_k = 0, d_k = 0, d_kp1 = next_c(d_k, d_t); k < i; ++k, mu_k = next_c(mu_k, mu_t), d_k = d_kp1, d_kp1 = next_c(d_k, d_t)) {
						base = mu[mu_k] < 0 ? d[d_kp1] : d[d_k];
						zi *= (base - d[d_i] + mu[mu_k]) * (base + d[d_i] + mu[mu_k]) / (d[d_k] - d[d_i]) / (d[d_k] + d[d_i]);
					}

					for (Size k = i, mu_k = at(0, k, mu_t), d_k = at(0, k, d_t), d_kp1 = next_c(d_k, d_t); k < mn - 1; ++k, mu_k = next_c(mu_k, mu_t), d_k = d_kp1, d_kp1 = next_c(d_k, d_t)) {
						base = mu[mu_k] < 0 ? d[d_kp1] : d[d_k];
						zi *= (base - d[d_i] + mu[mu_k]) * (base + d[d_i] + mu[mu_k]) / (d[d_kp1] - d[d_i]) / (d[d_kp1] + d[d_i]);
					}

					z[z_i] = s_sgn2(z[z_i]) * std::sqrt(zi);
				}

				pre_out = DvcPreOut{
					mn, 
					dn,
					ret1.q01, ret2.q01,
					q2_0, q1_e,
					r0, c0, s0,
					std::array<double,3>{0.0, 0.0, 0.0},
					ret1.mn, ret2.mn,
				};
			}

			return pre_out;
		};

		const auto dvc = [
#ifdef ARIS_DEBUG_DYNAMIC_SVD
			&
#endif
		](Size n, double *U, auto u_t, double*q, auto q_t, double *S, auto s_t, double *V, auto v_t, const auto&dvc, const auto& dvc_pre)->DvcPreOut
		{
			auto pre_out = dvc_pre(n, S, s_t, V, v_t, dvc, dvc_pre);

			const Size MAX_N = 9;

			if (n == 1) {
				auto& a1 = S[at(0, 0, s_t)];
				auto& a2 = S[at(1, 0, s_t)];

				auto theta = std::atan2(a2, a1);
				S[at(0, 0, s_t)] = std::sqrt(a1 * a1 + a2 * a2);

				U[at(0, 0, u_t)] = std::cos(theta);
				U[at(1, 0, u_t)] = std::sin(theta);

				pre_out.q01[0] = -std::sin(theta);// U_0_1
				pre_out.q01[1] = std::cos(theta);// U_1_1

#ifdef ARIS_DEBUG_DYNAMIC_SVD
				// make U //
				std::vector<double> _D_U((n + 1)* (n + 1));
				_D_U[at(0, 0, n + 1)] = U[at(0, 0, u_t)];
				_D_U[at(0, 1, n + 1)] = pre_out.q01[0];
				_D_U[at(1, 0, n + 1)] = U[at(1, 0, u_t)];
				_D_U[at(1, 1, n + 1)] = pre_out.q01[1];

				// make V //
				std::vector<double> _D_V(n* n);
				_D_V[at(0, 0, n)] = 1.0;

				// make S //
				std::vector<double> _D_S((n + 1) * n);
				_D_S[at(0, 0, n)] = S[at(0, 0, s_t)];
				_D_S[at(1, 0, n)] = 0.0;

				// verify result //
				std::vector<double> _D_Result((n + 1) * n), _D_Tem((n + 1) * n);
				s_mm(n + 1, n, n + 1, _D_U.data(), _D_S.data(), _D_Tem.data());
				s_mm(n + 1, n, n, _D_Tem.data(), n, _D_V.data(), T(n), _D_Result.data(), n);

				std::vector<double> _D_result_compare((n + 1) * n, 0.0);
				for (Size i(-1); ++i < n;) {
					_D_result_compare[at(i, i, n)] = _D_list.back().S[at(0, i, n)];
					_D_result_compare[at(i + 1, i, n)] = _D_list.back().S[at(1, i, n)];
				}

				if (!s_is_equal(n + 1, n, _D_Result.data(), _D_result_compare.data(), 1e-10)) {
					std::cout << "debug error" << std::endl;

					dsp(n + 1, n, _D_Result.data());
					dsp(n + 1, n, _D_result_compare.data());
				}

				_D_list.pop_back();
				if (!_D_list.empty())_D_list.back().num = 1;
#endif

			}
			else if (n == 2) {
				auto theta1 = std::atan2(S[at(1, 0, s_t)], S[at(0, 0, s_t)]);
				auto c1 = std::cos(theta1);
				auto s1 = std::sin(theta1);

				auto theta2 = std::atan2(S[at(1, 1, s_t)], c1 * S[at(0, 1, s_t)]);
				auto c2 = std::cos(theta2);
				auto s2 = std::sin(theta2);

				auto a = c1 * S[at(0, 0, s_t)] + s1 * S[at(1, 0, s_t)];
				auto b = s1 * S[at(0, 1, s_t)];
				auto d = c2 * c1 * S[at(0, 1, s_t)] + s2 * S[at(1, 1, s_t)];

				auto theta3 = 0.5 * std::atan2(2 * b * d, a * a + b * b - d * d);
				auto c3 = std::cos(theta3);
				auto s3 = std::sin(theta3);

				// S // 
				auto S1 = a * a + b * b + d * d;
				auto S2 = std::sqrt((a * a + b * b - d * d) * (a * a + b * b - d * d) + 4 * b * b * d * d);

				S[at(0, 0, s_t)] = std::sqrt((S1 + S2) / 2);
				S[at(0, 1, s_t)] = std::sqrt((S1 - S2) / 2);

				auto phi = 0.5 * std::atan2(2 * a * b, a * a - b * b - d * d);
				auto c_p = std::cos(phi);
				auto s_p = std::sin(phi);
				auto s11 = (a * c3) * c_p + (b * c3 + d * s3) * s_p;
				auto s22 = (a * s3) * s_p + (-b * s3 + d * c3) * c_p;

				// U //
				U[at(0, 0, u_t)] = s_sgn2(s11) * (c1 * c3 - c2 * s1 * s3);
				U[at(0, 1, u_t)] = s_sgn2(s22) * (-c1 * s3 - c2 * c3 * s1);
				U[at(1, 0, u_t)] = s_sgn2(s11) * (c3 * s1 + c1 * c2 * s3);
				U[at(1, 1, u_t)] = s_sgn2(s22) * (c1 * c2 * c3 - s1 * s3);
				U[at(2, 0, u_t)] = s_sgn2(s11) * s2 * s3;
				U[at(2, 1, u_t)] = s_sgn2(s22) * c3 * s2;

				pre_out.q01[0] = (s1 * s2);
				pre_out.q01[1] = (-c1 * s2);
				pre_out.q01[2] = c2;

				// V //
				V[at(0, 0, v_t)] = c_p;
				V[at(0, 1, v_t)] = -s_p;

#ifdef ARIS_DEBUG_DYNAMIC_SVD
				// make U //
				std::vector<double> _D_U((n + 1)* (n + 1));
				_D_U[at(0, 0, n + 1)] = U[at(0, 0, u_t)];
				_D_U[at(0, 1, n + 1)] = U[at(0, 1, u_t)];
				_D_U[at(0, 2, n + 1)] = pre_out.q01[0];
				_D_U[at(1, 0, n + 1)] = U[at(1, 0, u_t)];
				_D_U[at(1, 1, n + 1)] = U[at(1, 1, u_t)];
				_D_U[at(1, 2, n + 1)] = pre_out.q01[1];
				_D_U[at(2, 0, n + 1)] = U[at(2, 0, u_t)];
				_D_U[at(2, 1, n + 1)] = U[at(2, 1, u_t)];
				_D_U[at(2, 2, n + 1)] = pre_out.q01[2];

				// make V //
				std::vector<double> _D_V(n* n);
				_D_V[at(0, 0, n)] = V[at(0, 0, v_t)];
				_D_V[at(0, 1, n)] = V[at(0, 1, v_t)];
				_D_V[at(1, 0, n)] = -_D_V[at(0, 1, n)];
				_D_V[at(1, 1, n)] = _D_V[at(0, 0, n)];

				// make S //
				std::vector<double> _D_S((n + 1)* n, 0.0);
				_D_S[at(0, 0, n)] = S[at(0, 0, s_t)];
				_D_S[at(1, 1, n)] = S[at(0, 1, s_t)];

				// verify result //
				std::vector<double> _D_Result((n + 1)* n), _D_Tem((n + 1)* n);
				s_mm(n + 1, n, n + 1, _D_U.data(), _D_S.data(), _D_Tem.data());
				s_mm(n + 1, n, n, _D_Tem.data(), n, _D_V.data(), T(n), _D_Result.data(), n);

				std::vector<double> _D_result_compare((n + 1)* n, 0.0);
				for (Size i(-1); ++i < n;) {
					_D_result_compare[at(i, i, n)] = _D_list.back().S[at(0, i, n)];
					_D_result_compare[at(i + 1, i, n)] = _D_list.back().S[at(1, i, n)];
				}

				if (!s_is_equal(n + 1, n, _D_Result.data(), _D_result_compare.data(), 1e-10)) {
					std::cout << "debug error:" << std::endl;
					std::cout << "debug level:" << _D_list.back().level << "  num:" << _D_list.back().num << std::endl;

					std::cout << "dvc U:" << std::endl;
					dsp(n + 1, n + 1, _D_U.data());
					std::cout << "dvc S:" << std::endl;
					dsp(n + 1, n, _D_S.data());
					std::cout << "dvc V:" << std::endl;
					dsp(n, n, _D_V.data());

					std::cout << "dvc result:" << std::endl;
					dsp(n + 1, n, _D_Result.data());

					std::cout << "dvc result should be:" << std::endl;
					dsp(n + 1, n, _D_result_compare.data());
				}

				_D_list.pop_back();
				if (!_D_list.empty())_D_list.back().num = 1;
#endif
			}
			else if (n < MAX_N){
				auto h = n / 2;
				auto U1 = V + at(0, 0, v_t);
				auto u1_t = v_t;
				auto S1 = S + at(0, 0, s_t);
				auto s1_t = s_t;
				auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
				auto v1_t = v_t;
				auto U2 = V + at(h - 1, h, v_t);
				auto u2_t = v_t;
				auto S2 = S + at(0, h + 1, s_t);
				auto s2_t = s_t;
				auto V2 = V + at(h + 1, 0, v_t);
				auto v2_t = v_t;

				auto q1 = V + (h == n - h ? at(h + 1, h - 1, v_t) : at(0, h, v_t));
				auto q2 = U2 + at(0, n - h - 1, u2_t);

				auto q1_01 = pre_out.q1_01;
				auto q2_01 = pre_out.q2_01;
				auto c0 = pre_out.c0;
				auto s0 = pre_out.s0;
				auto mn = pre_out.mn;
				auto dn = pre_out.dn;

				auto d = S;
				auto d_t = s_t;
				auto z = S + at(1, 0, s_t);
				auto z_t = s_t;
				auto mu = V + at(n - 1, 0, v_t);
				auto mu_t = v_t;
				auto p = S + at(2, 0, s_t);
				auto p_t = s_t;

				//STEP 3：计算U
				auto ui = S + at(3, 0, s_t);
				auto ui_t = T(s_t);

				auto q1_012_sz = std::min(h + 1, Size(3));
				auto q2_012_sz = std::min(n - h, Size(3));

				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

						for (Size j = 0; j < n; ++j) {
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);
							ui[uij] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
						}
					}
					else {
						s_fill(n, 1, 0.0, ui, ui_t);
						auto uii = at(static_cast<Size>(p[at(0, i, p_t)]), 0, ui_t);
						ui[uii] = 1.0;
					}

					// apply deflation
					for (Size j = mn; j < n; ++j) {
						if (mu[at(0, j, mu_t)] == -2) {
						}
						else if (mu[at(0, j, mu_t)] == -1) {
							auto ui0 = at(static_cast<Size>(p[at(0, 0, p_t)]), 0, ui_t);
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);

							// type 3
							auto uia = std::cos(z[at(0, j, z_t)]) * ui[ui0] - std::sin(z[at(0, j, z_t)]) * ui[uij];
							auto uib = std::sin(z[at(0, j, z_t)]) * ui[ui0] + std::cos(z[at(0, j, z_t)]) * ui[uij];
							ui[ui0] = uia;
							ui[uij] = uib;
						}
						else {
							// type 4
							auto idx = static_cast<Size>(mu[at(0, j, mu_t)]);
							auto uik = at(static_cast<Size>(p[at(0, idx, p_t)]), 0, ui_t);
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);

							auto uia = std::cos(z[at(0, j, z_t)]) * ui[uik] - std::sin(z[at(0, j, z_t)]) * ui[uij];
							auto uib = std::sin(z[at(0, j, z_t)]) * ui[uik] + std::cos(z[at(0, j, z_t)]) * ui[uij];
							ui[uik] = uia;
							ui[uij] = uib;
						}
					}

					if (i < mn) {
						s_nm(n, 1, 1.0 / s_norm(n, ui, ui_t), ui, ui_t);
					}

					// 生成 U，基于U1 //
					s_mc(q1_012_sz, 1, c0 * ui[at(0, 0, ui_t)], pre_out.q1_01.data(), 1, U + at(0, i, u_t), u_t);
					s_mc(h + 1 - q1_012_sz, 1, c0 * ui[at(0, 0, ui_t)], q1, u1_t, U + at(q1_012_sz, i, u_t), u_t);
					s_mma(h + 1, 1, h, U1 + at(0, 0, u1_t), u1_t, ui + at(1, 0, ui_t), ui_t, U + at(0, i, u_t), u_t);

					// 生成 U，基于U2 //
					s_mc(q2_012_sz, 1, s0 * ui[at(0, 0, ui_t)], pre_out.q2_01.data(), 1, U + at(h + 1, i, u_t), u_t);
					s_mc(n - h - q2_012_sz, 1, s0 * ui[at(0, 0, ui_t)], q2, u2_t, U + at(h + 1 + q2_012_sz, i, u_t), u_t);
					s_mma(n - h, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + at(h + 1, 0, ui_t), ui_t, U + at(h + 1, i, u_t), u_t);
				}

				// 生成 q //
				s_mc(q1_012_sz, 1, -s0, pre_out.q1_01.data(), 1, pre_out.q01.data(), 1);

				if (q1_012_sz == 2) {
					pre_out.q01[2] = c0 * pre_out.q2_01[0];
					s_mc(q2_012_sz - 1, 1, c0, pre_out.q2_01.data() + 1, 1, q + at(0, 0, u_t), u_t);
					s_mc(n - h - q2_012_sz, 1, c0, q2, u2_t, q + at(q2_012_sz - 1, 0, u_t), u_t);
				}
				else {
					s_mc(h + 1 - q1_012_sz, 1, -s0, q1, u1_t, q + at(0, 0, u_t), u_t);
					s_mc(q2_012_sz, 1, c0, pre_out.q2_01.data(), 1, q + at(h + 1 - 3, 0, u_t), u_t);
					s_mc(n - h - q2_012_sz, 1, c0, q2, u2_t, q + at(h + 1 + q2_012_sz - 3, 0, u_t), u_t);
				}

				// step 7：复原V1 & V2到正确位置
				auto v_data = S + at(3, 0, s_t);
				auto v_data_t = s_t;

				// 因为前面压缩了一行，所以这里复原 V1 到新的内存位置 //
				if (h == 1) {
					v_data[at(0, 0, v_data_t)] = 1.0;
				}
				else if (h == 2) {
					s_mc(1, 2, V1, v1_t, v_data + at(0, 0, v_data_t), v_data_t);
					v_data[at(1, 0, v_data_t)] = -v_data[at(0, 1, v_data_t)];
					v_data[at(1, 1, v_data_t)] = v_data[at(0, 0, v_data_t)];
				}
				else {
					s_mc(h / 2, h, V1, v1_t, v_data + at(0, 0, v_data_t), v_data_t);
					s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, v_data + at(h / 2 + 1, 0, v_data_t), v_data_t);
					s_fill(1, pre_out.mn1, -1.0, v_data + at(h / 2, 0, v_data_t), v_data_t);
					s_fill(1, h - pre_out.mn1, 0.0, v_data + at(h / 2, pre_out.mn1, v_data_t), v_data_t);
					if (pre_out.mn1 == 0)v_data[at(h / 2, 0, v_data_t)] = 1.0;

					for (Size i = -1; ++i < h;) {
						s_nm(h, 1, 1.0 / s_norm(h, v_data + at(0, i, v_data_t), v_data_t), v_data + at(0, i, v_data_t), v_data_t);
					}
				}

				// 因为前面压缩了一行，所以这里复原 V2 到新的内存位置 //
				if (n - h - 1 == 1) {
					v_data[at(0, h, v_data_t)] = 1.0;
				}
				else if (n - h - 1 == 2) {
					s_mc(1, 2, V2, v2_t, v_data + at(0, h, v_data_t), v_data_t);
					v_data[at(1, h + 0, v_data_t)] = -v_data[at(0, h + 1, v_data_t)];
					v_data[at(1, h + 1, v_data_t)] = v_data[at(0, h + 0, v_data_t)];
				}
				else {
					auto k = n - h - 1;
					s_mc(k / 2, k, V2, v2_t, v_data + at(0, h, v_data_t), v_data_t);
					s_mc(k - k / 2 - 1, k, V2 + at(k / 2, 0, v2_t), v2_t, v_data + at(k / 2 + 1, h, v_data_t), v_data_t);
					s_fill(1, pre_out.mn2, -1.0, v_data + at(k / 2, h, v_data_t), v_data_t);
					s_fill(1, k - pre_out.mn2, 0.0, v_data + at(k / 2, h + pre_out.mn2, v_data_t), v_data_t);
					if (pre_out.mn2 == 0)v_data[at(k / 2, h + pre_out.mn2, v_data_t)] = 1.0;

					for (Size i = -1; ++i < k;) {
						s_nm(k, 1, 1.0 / s_norm(k, v_data + at(0, i + h, v_data_t), v_data_t), v_data + at(0, i + h, v_data_t), v_data_t);
					}
				}

				// 使用复原后的 V1 & V2
				V1 = v_data;
				V2 = v_data + at(0, h, v_data_t);
				auto v1_t2 = v_data_t;
				auto v2_t2 = v_data_t;

				//step 8：计算V
				// 若 n < 5，此时将p暂存至局部变量中
				double p_tem[5];
				if (n < 5)	s_mc(1, 5, p, p_t, p_tem, 5);

				// 此时vi的位置也可能会变化
				auto vi = n < 5 ? S + at(2, 0, s_t) : S + at(n, 0, s_t);
				auto vi_t = T(s_t);

				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

						for (Size j = 1; j < n; ++j) {
							vi[at(j, 0, vi_t)] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
						}
					}
					else {
						s_fill(n, 1, 0.0, vi, vi_t);
						vi[at(i, 0, vi_t)] = 1.0;
					}

					// apply deflation
					for (Size j = mn; j < n; ++j) {
						if (mu[at(0, j, mu_t)] == -2) {
						}
						else if (mu[at(0, j, mu_t)] == -1) {
							// type 3
						}
						else {
							// type 4
							auto via = std::cos(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] - std::sin(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
							auto vib = std::sin(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] + std::cos(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
							vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] = via;
							vi[at(j, 0, vi_t)] = vib;
						}
					}

					// apply permutation
					if (n < 5)
						s_permutate_inv(n, 1, RowIterator<Size>(p_tem, 5), vi, vi_t);
					else
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), vi, vi_t);

					// 左乘V1 V2
					s_mm(h, 1, h, V1, v1_t2, vi + at(1, 0, vi_t), vi_t, V + at(0, i, v_t), v_t);
					s_mm(n - h - 1, 1, n - h - 1, V2, v2_t2, vi + at(h + 1, 0, vi_t), vi_t, V + at(h, i, v_t), v_t);
				}

				//step 8：计算S
				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];
						S[at(0, i, s_t)] = base + mu[at(0, i, mu_t)];
					}
					else {
						S[at(0, i, s_t)] = d[at(0, i, d_t)];
					}
				}

	#ifdef ARIS_DEBUG_DYNAMIC_SVD
				// make U //
				std::vector<double> _D_U((n + 1)* (n + 1));
				s_mc(n + 1, n, U, u_t, _D_U.data(), n + 1);
				s_mc(3, 1, pre_out.q01.data(), 1, _D_U.data() + n, n + 1);
				s_mc(n + 1 - 3, 1, q, u_t, _D_U.data() + at(3, n, n + 1), n + 1);

				// make V //
				std::vector<double> _D_V(n* n, 0.0);
				s_mc(n / 2, n, V, v_t, _D_V.data() + at(0, 0, n), n);
				s_mc(n - n / 2 - 1, n, V + at(n / 2, 0, v_t), v_t, _D_V.data() + at(n / 2 + 1, 0, n), n);
				s_fill(1, mn, -1.0, _D_V.data() + at(n / 2, 0, n), n);
				s_fill(1, n - mn, 0.0, _D_V.data() + at(n / 2, mn, n), n);
				if (mn == 0)_D_V.data()[at(n / 2, 0, n)] = 1.0;

				for (Size i = -1; ++i < n;) {
					s_nm(n, 1, 1.0 / s_norm(n, _D_V.data() + at(0, i, n), n), _D_V.data() + at(0, i, n), n);
				}

				// make S //
				std::vector<double> _D_S((n + 1) * n);
				s_fill(n + 1, n, 0.0, _D_S.data(), n);
				for (Size i(-1); ++i < n;) {
					_D_S.data()[at(i, i, n)] = S[at(0, i, s_t)];
				}

				std::vector<double> _D_Result((n + 1)* n), _D_Tem((n + 1)* n);
				s_mm(n + 1, n, n + 1, _D_U.data(), _D_S.data(), _D_Tem.data());
				s_mm(n + 1, n, n, _D_Tem.data(), n, _D_V.data(), T(n), _D_Result.data(), n);

				std::vector<double> _D_result_compare((n + 1)*n, 0.0);
				for (Size i(-1); ++i < n;) {
					_D_result_compare[at(i, i, n)] = _D_list.back().S[at(0, i, n)];
					_D_result_compare[at(i + 1, i, n)] = _D_list.back().S[at(1, i, n)];
				}

				if (!s_is_equal(n + 1, n, _D_Result.data(), _D_result_compare.data(), 1e-10)) {
					std::cout << "debug error:" << std::endl;
					std::cout << "debug level:" << _D_list.back().level << "  num:" << _D_list.back().num << std::endl;

					std::cout << "dvc U:" << std::endl;
					dsp(n + 1, n + 1, _D_U.data());
					std::cout << "dvc S:" << std::endl;
					dsp(n + 1, n, _D_S.data());
					std::cout << "dvc V:" << std::endl;
					dsp(n, n, _D_V.data());

					std::cout << "dvc result:" << std::endl;
					dsp(n + 1, n, _D_Result.data());

					std::cout << "dvc result should be:" << std::endl;
					dsp(n + 1, n, _D_result_compare.data());

					std::exit(0);
				}

				_D_list.pop_back();
				if (!_D_list.empty())_D_list.back().num = 1;
	#endif
			}
			else {
				auto h = n / 2;
				auto U1 = V + at(0, 0, v_t);
				auto u1_t = v_t;
				auto S1 = S + at(0, 0, s_t);
				auto s1_t = s_t;
				auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
				auto v1_t = v_t;
				auto U2 = V + at(h - 1, h, v_t);
				auto u2_t = v_t;
				auto S2 = S + at(0, h + 1, s_t);
				auto s2_t = s_t;
				auto V2 = V + at(h + 1, 0, v_t);
				auto v2_t = v_t;

				auto q1 = V + (h == n - h ? at(h + 1, h - 1, v_t) : at(0, h, v_t));
				auto q2 = U2 + at(0, n - h - 1, u2_t);

				auto q1_01 = pre_out.q1_01;
				auto q2_01 = pre_out.q2_01;
				auto c0 = pre_out.c0;
				auto s0 = pre_out.s0;
				auto mn = pre_out.mn;
				auto dn = pre_out.dn;

				auto d = S;
				auto d_t = s_t;
				auto z = S + at(1, 0, s_t);
				auto z_t = s_t;
				auto mu = V + at(n - 1, 0, v_t);
				auto mu_t = v_t;
				auto p = S + at(2, 0, s_t);
				auto p_t = s_t;

				//STEP 3：计算U
				auto ui = S + at(3, 0, s_t);
				auto ui_t = T(s_t);

				auto q1_012_sz = std::min(h + 1, Size(3));
				auto q2_012_sz = std::min(n - h, Size(3));

				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

						for (Size j = 0; j < n; ++j) {
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);
							ui[uij] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
						}
					}
					else {
						s_fill(n, 1, 0.0, ui, ui_t);
						auto uii = at(static_cast<Size>(p[at(0, i, p_t)]), 0, ui_t);
						ui[uii] = 1.0;
					}

					// apply deflation
					for (Size j = mn; j < n; ++j) {
						if (mu[at(0, j, mu_t)] == -2) {
						}
						else if (mu[at(0, j, mu_t)] == -1) {
							auto ui0 = at(static_cast<Size>(p[at(0, 0, p_t)]), 0, ui_t);
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);
							
							// type 3
							auto uia = std::cos(z[at(0, j, z_t)]) * ui[ui0] - std::sin(z[at(0, j, z_t)]) * ui[uij];
							auto uib = std::sin(z[at(0, j, z_t)]) * ui[ui0] + std::cos(z[at(0, j, z_t)]) * ui[uij];
							ui[ui0] = uia;
							ui[uij] = uib;
						}
						else {
							// type 4
							auto idx = static_cast<Size>(mu[at(0, j, mu_t)]);
							auto uik = at(static_cast<Size>(p[at(0, idx, p_t)]), 0, ui_t);
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);

							auto uia = std::cos(z[at(0, j, z_t)]) * ui[uik] - std::sin(z[at(0, j, z_t)]) * ui[uij];
							auto uib = std::sin(z[at(0, j, z_t)]) * ui[uik] + std::cos(z[at(0, j, z_t)]) * ui[uij];
							ui[uik] = uia;
							ui[uij] = uib;
						}
					}

					if (i < mn) {
						s_nm(n, 1, 1.0 / s_norm(n, ui, ui_t), ui, ui_t);
					}

					// 生成 U，基于U1 //
					s_mc(q1_012_sz,         1, c0 * ui[at(0, 0, ui_t)], pre_out.q1_01.data(), 1,    U + at(0, i, u_t),         u_t);
					s_mc(h + 1 - q1_012_sz, 1, c0 * ui[at(0, 0, ui_t)],                   q1, u1_t, U + at(q1_012_sz, i, u_t), u_t);
					s_mma(h + 1,  1,         h, U1 + at(0, 0, u1_t), u1_t, ui + at(1, 0, ui_t), ui_t, U + at(0, i, u_t), u_t);

					// 生成 U，基于U2 //
					s_mc(q2_012_sz, 1, s0 * ui[at(0, 0, ui_t)], pre_out.q2_01.data(), 1, U + at(h + 1, i, u_t), u_t);
					s_mc(n - h - q2_012_sz, 1, s0 * ui[at(0, 0, ui_t)], q2, u2_t, U + at(h + 1 + q2_012_sz, i, u_t), u_t);
					s_mma(n - h, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + at(h + 1, 0, ui_t), ui_t, U + at(h + 1, i, u_t), u_t);
				}

				// 生成 q //
				s_mc(q1_012_sz, 1, -s0, pre_out.q1_01.data(), 1, pre_out.q01.data(), 1);
				
				if (q1_012_sz == 2) {
					pre_out.q01[2] = c0 * pre_out.q2_01[0];
					s_mc(q2_012_sz - 1, 1, c0, pre_out.q2_01.data() + 1, 1, q + at(0, 0, u_t), u_t);
					s_mc(n - h - q2_012_sz, 1, c0, q2, u2_t, q + at(q2_012_sz - 1, 0, u_t), u_t);
				}
				else {
					s_mc(h + 1 - q1_012_sz, 1, -s0, q1, u1_t, q + at(0, 0, u_t), u_t);
					s_mc(q2_012_sz, 1, c0, pre_out.q2_01.data(), 1, q + at(h + 1 - 3, 0, u_t), u_t);
					s_mc(n - h - q2_012_sz, 1, c0, q2, u2_t, q + at(h + 1 + q2_012_sz - 3, 0, u_t), u_t);
				}

				// step 7：复原V1 & V2到正确位置
				auto v_data = S + at(3, 0, s_t);
				auto v_data_t = s_t;

				// 因为前面压缩了一行，所以这里复原 V1 到新的内存位置 //
				if (h == 1) {
					v_data[at(0, 0, v_data_t)] = 1.0;
				}
				else if (h == 2) {
					s_mc(1, 2, V1, v1_t, v_data + at(0, 0, v_data_t), v_data_t);
					v_data[at(1, 0, v_data_t)] = -v_data[at(0, 1, v_data_t)];
					v_data[at(1, 1, v_data_t)] = v_data[at(0, 0, v_data_t)];
				}
				else {
					s_mc(h / 2, h, V1, v1_t, v_data + at(0, 0, v_data_t), v_data_t);
					s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, v_data + at(h / 2 + 1, 0, v_data_t), v_data_t);
					s_fill(1, pre_out.mn1, -1.0, v_data + at(h / 2, 0, v_data_t), v_data_t);
					s_fill(1, h - pre_out.mn1, 0.0, v_data + at(h / 2, pre_out.mn1, v_data_t), v_data_t);
					if (pre_out.mn1 == 0)v_data[at(h / 2, 0, v_data_t)] = 1.0;

					for (Size i = -1; ++i < h;) {
						s_nm(h, 1, 1.0 / s_norm(h, v_data + at(0, i, v_data_t), v_data_t), v_data + at(0, i, v_data_t), v_data_t);
					}
				}

				// 因为前面压缩了一行，所以这里复原 V2 到新的内存位置 //
				if (n - h - 1 == 1) {
					v_data[at(0, h, v_data_t)] = 1.0;
				}
				else if (n - h - 1 == 2) {
					s_mc(1, 2, V2, v2_t, v_data + at(0, h, v_data_t), v_data_t);
					v_data[at(1, h + 0, v_data_t)] = -v_data[at(0, h + 1, v_data_t)];
					v_data[at(1, h + 1, v_data_t)] = v_data[at(0, h + 0, v_data_t)];
				}
				else {
					auto k = n - h - 1;
					s_mc(k / 2, k, V2, v2_t, v_data + at(0, h, v_data_t), v_data_t);
					s_mc(k - k / 2 - 1, k, V2 + at(k / 2, 0, v2_t), v2_t, v_data + at(k / 2 + 1, h, v_data_t), v_data_t);
					s_fill(1, pre_out.mn2, -1.0, v_data + at(k / 2, h, v_data_t), v_data_t);
					s_fill(1, k - pre_out.mn2, 0.0, v_data + at(k / 2, h + pre_out.mn2, v_data_t), v_data_t);
					if (pre_out.mn2 == 0)v_data[at(k / 2, h + pre_out.mn2, v_data_t)] = 1.0;

					for (Size i = -1; ++i < k;) {
						s_nm(k, 1, 1.0 / s_norm(k, v_data + at(0, i + h, v_data_t), v_data_t), v_data + at(0, i + h, v_data_t), v_data_t);
					}
				}

				// 使用复原后的 V1 & V2
				V1 = v_data;
				V2 = v_data + at(0, h, v_data_t);
				auto v1_t2 = v_data_t;
				auto v2_t2 = v_data_t;

				//step 8：计算V
				// 若 n < 5，此时将p暂存至局部变量中
				double p_tem[5];
				if (n < 5)	s_mc(1, 5, p, p_t, p_tem, 5);
				
				// 此时vi的位置也可能会变化
				auto vi = n < 5 ? S + at(2, 0, s_t) : S + at(n, 0, s_t);
				auto vi_t = T(s_t);

				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

						for (Size j = 1; j < n; ++j) {
							vi[at(j, 0, vi_t)] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
						}
					}
					else {
						s_fill(n, 1, 0.0, vi, vi_t);
						vi[at(i, 0, vi_t)] = 1.0;
					}

					// apply deflation
					for (Size j = mn; j < n; ++j) {
						if (mu[at(0, j, mu_t)] == -2) {
						}
						else if (mu[at(0, j, mu_t)] == -1) {
							// type 3
						}
						else {
							// type 4
							auto via = std::cos(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] - std::sin(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
							auto vib = std::sin(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] + std::cos(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
							vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] = via;
							vi[at(j, 0, vi_t)] = vib;
						}
					}

					// apply permutation
					if(n < 5)
						s_permutate_inv(n, 1, RowIterator<Size>(p_tem, 5), vi, vi_t);
					else
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), vi, vi_t);

					// 左乘V1 V2
					s_mm(h, 1, h, V1, v1_t2, vi + at(1, 0, vi_t), vi_t, V + at(0, i, v_t), v_t);
					s_mm(n - h - 1, 1, n - h - 1, V2, v2_t2, vi + at(h + 1, 0, vi_t), vi_t, V + at(h, i, v_t), v_t);
				}

				//step 8：计算S
				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];
						S[at(0, i, s_t)] = base + mu[at(0, i, mu_t)];
					}
					else {
						S[at(0, i, s_t)] = d[at(0, i, d_t)];
					}
				}

#ifdef ARIS_DEBUG_DYNAMIC_SVD
				// make U //
				std::vector<double> _D_U((n + 1)* (n + 1));
				s_mc(n + 1, n, U, u_t, _D_U.data(), n + 1);
				s_mc(3, 1, pre_out.q01.data(), 1, _D_U.data() + n, n + 1);
				s_mc(n + 1 - 3, 1, q, u_t, _D_U.data() + at(3, n, n + 1), n + 1);

				// make V //
				std::vector<double> _D_V(n* n, 0.0);
				s_mc(n / 2, n, V, v_t, _D_V.data() + at(0, 0, n), n);
				s_mc(n - n / 2 - 1, n, V + at(n / 2, 0, v_t), v_t, _D_V.data() + at(n / 2 + 1, 0, n), n);
				s_fill(1, mn, -1.0, _D_V.data() + at(n / 2, 0, n), n);
				s_fill(1, n - mn, 0.0, _D_V.data() + at(n / 2, mn, n), n);
				if (mn == 0)_D_V.data()[at(n / 2, 0, n)] = 1.0;

				for (Size i = -1; ++i < n;) {
					s_nm(n, 1, 1.0 / s_norm(n, _D_V.data() + at(0, i, n), n), _D_V.data() + at(0, i, n), n);
				}

				// make S //
				std::vector<double> _D_S((n+1) * n);
				s_fill(n+1, n, 0.0, _D_S.data(), n);
				for (Size i(-1); ++i < n;) {
					_D_S.data()[at(i, i, n)] = S[at(0, i, s_t)];
				}

				std::vector<double> _D_Result((n + 1)* n), _D_Tem((n + 1)* n);
				s_mm(n + 1, n, n + 1, _D_U.data(), _D_S.data(), _D_Tem.data());
				s_mm(n + 1, n, n, _D_Tem.data(), n, _D_V.data(), T(n), _D_Result.data(), n);

				std::vector<double> _D_result_compare((n + 1)*n,0.0);
				for (Size i(-1); ++i < n;) {
					_D_result_compare[at(i, i, n)] = _D_list.back().S[at(0, i, n)];
					_D_result_compare[at(i+1, i, n)] = _D_list.back().S[at(1, i, n)];
				}

				if (!s_is_equal(n + 1, n, _D_Result.data(), _D_result_compare.data(), 1e-10)) {
					std::cout << "debug error:" << std::endl;
					std::cout << "debug level:"<<_D_list.back().level << "  num:"<< _D_list.back().num << std::endl;

					std::cout << "dvc U:" << std::endl;
					dsp(n + 1, n + 1, _D_U.data());
					std::cout << "dvc S:" << std::endl;
					dsp(n + 1, n, _D_S.data());
					std::cout << "dvc V:" << std::endl;
					dsp(n, n, _D_V.data());

					std::cout << "dvc result:" << std::endl;
					dsp(n + 1, n, _D_Result.data());

					std::cout << "dvc result should be:" << std::endl;
					dsp(n + 1, n, _D_result_compare.data());

					std::exit(0);
				}

				_D_list.pop_back();
				if (!_D_list.empty())_D_list.back().num = 1;
#endif
			}

			return pre_out;
		};

		//////////////////////////////////////////////////////////// PART 3: 首次调用 /////////////////////////////////////////
		const Size MAX_N = 9;
		
		if (n < MAX_N) {
			auto pre_out = dvc_pre(n, U, u_t, V, v_t, dvc, dvc_pre);
			s_mc(2, n, U, u_t, S, s_t);

			auto q = U + at(0, n, u_t);

			auto h = n / 2;
			auto U1 = V + at(0, 0, v_t);
			auto u1_t = v_t;
			auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
			auto v1_t = v_t;
			auto U2 = V + at(h - 1, h, v_t);
			auto u2_t = v_t;
			auto V2 = V + at(h + 1, 0, v_t);
			auto v2_t = v_t;

			auto q1 = V + (h == n - h ? at(h + 1, h - 1, v_t) : at(0, h, v_t));
			auto q2 = U2 + at(0, n - h - 1, u2_t);

			auto c0 = pre_out.c0;
			auto s0 = pre_out.s0;
			auto mn = pre_out.mn;
			auto dn = pre_out.dn;

			auto d = S;
			auto d_t = s_t;
			auto z = S + at(1, 0, s_t);
			auto z_t = s_t;

			double mu[MAX_N];
			double p[MAX_N];
			double ui[MAX_N];
			double vi[MAX_N];
			
			// 初始化mu p
			s_mc(1, n, U + at(2, 0, u_t), u_t, p, MAX_N);
			s_mc(1, n, V + at(n - 1, 0, v_t), v_t, mu, MAX_N);

			// STEP 1: 计算U
			{
				// 计算 U //
				{
					auto q1_012_sz = std::min(h + 1, Size(3));
					auto q2_012_sz = std::min(n - h, Size(3));

					// 计算ui所产生的U（不经过householder变换）
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[i] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 0; j < n; ++j) {
								ui[j] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[i]) / (d[at(0, j, d_t)] + base + mu[i]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, ui);
							ui[i] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[j] == -2) {
							}
							else if (mu[j] == -1) {
								// type 3
								auto ui0 = 0;
								auto uij = j;

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[ui0] - std::sin(z[at(0, j, z_t)]) * ui[uij];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[ui0] + std::cos(z[at(0, j, z_t)]) * ui[uij];
								ui[ui0] = uia;
								ui[uij] = uib;
							}
							else {
								// type 4
								auto k = static_cast<Size>(mu[j]);
								auto uik = k;
								auto uij = j;

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[uik] - std::sin(z[at(0, j, z_t)]) * ui[uij];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[uik] + std::cos(z[at(0, j, z_t)]) * ui[uij];
								ui[uik] = uia;
								ui[uij] = uib;
							}
						}

						// apply permutation
						s_permutate_inv(n, 1, RowIterator<Size>(p, MAX_N), ui, 1);

						// 归一化
						if (i < mn) {
							s_nm(n, 1, 1.0 / s_norm(n, ui, 1), ui, 1);
						}

						// 生成 U，基于U1 //
						s_mc(q1_012_sz, 1, c0* ui[0], pre_out.q1_01.data(), 1, U + at(0, i, u_t), u_t);
						s_mc(h + 1 - q1_012_sz, 1, c0* ui[0], q1, u1_t, U + at(q1_012_sz, i, u_t), u_t);
						s_mma(h + 1, 1, h, U1 + at(0, 0, u1_t), u1_t, ui + 1, 1, U + at(0, i, u_t), u_t);

						// 生成 U，基于U2 //
						if (m > n) {
							auto ui0 = 0;
							s_mc(q2_012_sz, 1, s0 * ui[ui0], pre_out.q2_01.data(), 1, U + at(h + 1, i, u_t), u_t);
							s_mc(n - h - q2_012_sz, 1, s0 * ui[ui0], q2, u2_t, U + at(h + 1 + q2_012_sz, i, u_t), u_t);

							s_mma(n - h, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + h + 1, 1, U + at(h + 1, i, u_t), u_t);
						}
						else {
							auto minus_1 = n - h - q2_012_sz > 0 ? 0 : 1;
							auto minus_2 = n - h - q2_012_sz > 0 ? 1 : 0;

							auto ui0 = 0;
							s_mc(q2_012_sz - minus_1, 1, s0 * ui[ui0], pre_out.q2_01.data(), 1, U + at(h + 1, i, u_t), u_t);
							s_mc(n - h - q2_012_sz - minus_2, 1, s0 * ui[ui0], q2, u2_t, U + at(h + 1 + q2_012_sz, i, u_t), u_t);

							s_mma(n - h - 1, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + h + 1, 1, U + at(h + 1, i, u_t), u_t);
						}
					}

					// 计算q1&q2所产生的q //
					if (m > n) {
						s_mc(q1_012_sz, 1, -s0, pre_out.q1_01.data(), 1, q + at(0, 0, u_t), u_t);
						s_mc(h + 1 - q1_012_sz, 1, -s0, q1, u1_t, q + at(q1_012_sz, 0, u_t), u_t);

						s_mc(q2_012_sz, 1, c0, pre_out.q2_01.data(), 1, q + at(h + 1, 0, u_t), u_t);
						s_mc(n - h - q2_012_sz, 1, c0, q2, u2_t, q + at(h + 1 + q2_012_sz, 0, u_t), u_t);
					}
				}

				// 处理 m > n + 1时额外的方阵
				if (m > n + 1) {
					s_fill(m - n - 1, n + 1, 0.0, U + at(n + 1, 0, u_t), u_t);
					s_fill(n + 1, m - n - 1, 0.0, U + at(0, n + 1, u_t), u_t);
					s_eye(m - n - 1, U + at(n + 1, n + 1, u_t), u_t);
				}

				// 反 householder 变换 //
				s_householder_u_q_dot(m - 1, n, m, S + at(1, 0, s_t), s_t, U + at(1, 0, u_t), u_t, U + at(1, 0, u_t), u_t);
			}

			// STEP 2: 计算V
			{
				// 根据 vi 计算
				{
					// 因为前面压缩了一行，所以这里复原 V1 到新的内存位置 //
					double V1_new[9], V2_new[9];
					auto v1_t_new = h;
					auto v2_t_new = n - h - 1;

					if (h == 1) {
						V1_new[at(0, 0, 1)] = 1.0;
					}
					else if (h == 2) {
						s_mc(1, 2, V1, v1_t, V1_new, 2);
						V1_new[at(1, 0, 2)] = -V1_new[at(0, 1, 2)];
						V1_new[at(1, 1, 2)] = V1_new[at(0, 0, 2)];
					}
					else if (h == 3) {
						s_mc(h / 2, h, V1, v1_t, V1_new + at(0, 0, v1_t_new), v1_t_new);
						s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, V1_new + at(h / 2 + 1, 0, v1_t_new), v1_t_new);
						s_fill(1, pre_out.mn1, -1.0, V1_new + at(h / 2, 0, v1_t_new), v1_t_new);
						s_fill(1, h - pre_out.mn1, 0.0, V1_new + at(h / 2, pre_out.mn1, v1_t_new), v1_t_new);
						if (pre_out.mn1 == 0)V1_new[at(h / 2, 0, v1_t_new)] = 1.0;

						for (Size i = -1; ++i < h;) {
							s_nm(h, 1, 1.0 / s_norm(h, V1_new + at(0, i, v1_t_new), v1_t_new), V1_new + at(0, i, v1_t_new), v1_t_new);
						}
					}
					else {
						s_mc(h / 2, h, V1, v1_t, V1_new + at(0, 0, v1_t_new), v1_t_new);
						s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, V1_new + at(h / 2 + 1, 0, v1_t_new), v1_t_new);
						s_fill(1, pre_out.mn1, -1.0, V1_new + at(h / 2, 0, v1_t_new), v1_t_new);
						s_fill(1, h - pre_out.mn1, 0.0, V1_new + at(h / 2, pre_out.mn1, v1_t_new), v1_t_new);
						if (pre_out.mn1 == 0)V1_new[at(h / 2, 0, v1_t_new)] = 1.0;

						for (Size i = -1; ++i < h;) {
							s_nm(h, 1, 1.0 / s_norm(h, V1_new + at(0, i, v1_t_new), v1_t_new), V1_new + at(0, i, v1_t_new), v1_t_new);
						}
					}

					// 因为前面压缩了一行，所以这里复原 V2 到新的内存位置 //
					if (n - h - 1 == 1) {
						V2_new[at(0, 0, 1)] = 1.0;
					}
					else if (n - h - 1 == 2) {
						s_mc(1, 2, V2, v2_t, V2_new + at(0, 0, 2), 2);
						V2_new[at(1, 0, 2)] = -V2_new[at(0, 1, 2)];
						V2_new[at(1, 1, 2)] = V2_new[at(0, 0, 2)];
					}
					else {
						auto k = n - h - 1;
						s_mc(k / 2, k, V2, v2_t, V2_new + at(0, 0, v2_t_new), v2_t_new);
						s_mc(k - k / 2 - 1, k, V2 + at(k / 2, 0, v2_t), v2_t, V2_new + at(k / 2 + 1, 0, v2_t_new), v2_t_new);
						s_fill(1, pre_out.mn2, -1.0, V2_new + at(k / 2, 0, v2_t_new), v2_t_new);
						s_fill(1, k - pre_out.mn2, 0.0, V2_new + at(k / 2, pre_out.mn2, v2_t_new), v2_t_new);
						if (pre_out.mn2 == 0)V2_new[at(k / 2, pre_out.mn2, v2_t_new)] = 1.0;

						for (Size i = -1; ++i < k;) {
							s_nm(k, 1, 1.0 / s_norm(k, V2_new + at(0, i, v2_t_new), v2_t_new), V2_new + at(0, i, v2_t_new), v2_t_new);
						}
					}

					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[i] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 1; j < n; ++j) {
								vi[j] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[i]) / (d[at(0, j, d_t)] + base + mu[i]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, vi, 1);
							vi[i] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[j] == -2) {
							}
							else if (mu[j] == -1) {
								// type 3
							}
							else {
								// type 4
								auto via = std::cos(z[at(0, j, z_t)]) * vi[static_cast<Size>(mu[j])] - std::sin(z[at(0, j, z_t)]) * vi[j];
								auto vib = std::sin(z[at(0, j, z_t)]) * vi[static_cast<Size>(mu[j])] + std::cos(z[at(0, j, z_t)]) * vi[j];
								vi[static_cast<Size>(mu[j])] = via;
								vi[j] = vib;
							}
						}

						// apply permutation
						s_permutate_inv(n, 1, RowIterator<Size>(p, MAX_N), vi, 1);


						if (i < mn) {
							vi[0] = -1.0;
						}

						// 左乘V1
						s_mm(h, 1, h, V1_new, h, vi + 1, 1, V + at(0, i, v_t), v_t);

						// 左乘V2
						s_mm(n - h - 1, 1, n - h - 1, V2_new, n - h - 1, vi + h + 1, 1, V + at(h + 1, i, v_t), v_t);
					}
				}

				// 计算第h行
				for (Size i = 0; i < n; ++i) {
					V[at(h, i, v_t)] = i < mn ? -1.0 : (i == 0 ? 1.0 : 0.0);
					s_nm(n, 1, 1.0 / s_norm(n, V + at(0, i, v_t), v_t), V + at(0, i, v_t), v_t);
				}

				// 反 householder 变换 //
				s_householder_u_q_dot(2, 1, n, householder_vec_n, 1, V + at(n - 2, 0, v_t), v_t, V + at(n - 2, 0, v_t), v_t);
				s_householder_u_q_dot(n, n - 2, n, S + at(2, 0, s_t), T(s_t), V, v_t, V, v_t);
			}

			// STEP 3：计算S
			{
				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[i] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];
						S[at(0, i, s_t)] = base + mu[i];
					}
					else {
						S[at(0, i, s_t)] = d[at(0, i, d_t)];
					}
				}

				s_fill(m - 1, n, 0.0, S + at(1, 0, s_t), s_t);
				for (Size i = 0; ++i < n;) {
					S[at(i, i, s_t)] = S[at(0, i, s_t)];
					S[at(0, i, s_t)] = 0.0;
				}
			}
			

		
		}
		else {
			auto pre_out = dvc_pre(n, U, u_t, V, v_t, dvc, dvc_pre);
			s_mc(2, n, U, u_t, S, s_t);

			auto q = U + at(0, n, u_t);

			auto h = n / 2;
			auto U1 = V + at(0, 0, v_t);
			auto u1_t = v_t;
			auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
			auto v1_t = v_t;
			auto U2 = V + at(h - 1, h, v_t);
			auto u2_t = v_t;
			auto V2 = V + at(h + 1, 0, v_t);
			auto v2_t = v_t;

			auto q1 = V + (h == n - h ? at(h + 1, h - 1, v_t) : at(0, h, v_t));
			auto q2 = U2 + at(0, n - h - 1, u2_t);

			auto c0 = pre_out.c0;
			auto s0 = pre_out.s0;
			auto mn = pre_out.mn;
			auto dn = pre_out.dn;

			auto d = S;
			auto d_t = s_t;
			auto z = S + at(1, 0, s_t);
			auto z_t = s_t;

			// STEP 1: 计算U
			double r_of_right_corner[12];
			double r_of_tem_memory_for_v1v2[9]; // 为保存v1v2，将householder左下方变成方阵，需保存右上角
			{
				auto mu = V + at(n - 1, 0, v_t);
				auto mu_t = v_t;

				// 将 p 移动到 U 的第 n 行
				s_mc(1, n, U + at(2, 0, u_t), u_t, U + at(n - 2, 0, u_t), u_t);

				// 根据 U1 计算 //
				{
					auto q1_012_sz = std::min(h + 1, Size(3));

					auto p = U + at(n - 2, 0, u_t);
					auto p_t = u_t;
					auto ui = U + at(n - 1, 0, u_t);
					auto ui_t = T(u_t);

					// 计算U1所产生的U（不经过householder变换）
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 0; j < n; ++j) {
								ui[at(j, 0, ui_t)] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, ui, ui_t);
							ui[at(i, 0, ui_t)] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[at(0, j, mu_t)] == -2) {
							}
							else if (mu[at(0, j, mu_t)] == -1) {
								// type 3
								auto uia = std::cos(z[at(0, j, z_t)]) * ui[at(0, 0, ui_t)] - std::sin(z[at(0, j, z_t)]) * ui[at(j, 0, ui_t)];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[at(0, 0, ui_t)] + std::cos(z[at(0, j, z_t)]) * ui[at(j, 0, ui_t)];
								ui[at(0, 0, ui_t)] = uia;
								ui[at(j, 0, ui_t)] = uib;
							}
							else {
								// type 4
								auto idx = static_cast<Size>(mu[at(0, j, mu_t)]);

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[at(idx, 0, ui_t)] - std::sin(z[at(0, j, z_t)]) * ui[at(j, 0, ui_t)];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[at(idx, 0, ui_t)] + std::cos(z[at(0, j, z_t)]) * ui[at(j, 0, ui_t)];
								ui[at(idx, 0, ui_t)] = uia;
								ui[at(j, 0, ui_t)] = uib;
							}
						}

						// apply permutation
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), ui, ui_t);

						if (i < mn) {
							s_nm(n, 1, 1.0 / s_norm(n, ui, ui_t), ui, ui_t);
						}

						// 生成 U，基于U1 //
						s_mc(q1_012_sz, 1, c0* ui[at(0, 0, ui_t)], pre_out.q1_01.data(), 1, U + at(0, i, u_t), u_t);
						s_mc(h + 1 - q1_012_sz, 1, c0* ui[at(0, 0, ui_t)], q1, u1_t, U + at(q1_012_sz, i, u_t), u_t);
						s_mma(h + 1, 1, h, U1 + at(0, 0, u1_t), u1_t, ui + at(1, 0, ui_t), ui_t, U + at(0, i, u_t), u_t);
					}
					// 计算q1所产生的q //
					if (m > n) {
						s_mc(q1_012_sz, 1, -s0, pre_out.q1_01.data(), 1, q + at(0, 0, u_t), u_t);
						s_mc(h + 1 - q1_012_sz, 1, -s0, q1, u1_t, q + at(q1_012_sz, 0, u_t), u_t);
					}
				}

				// 根据 U2 计算 //
				{
					// 移动 V1 的第一行，为p腾出位置 //
					// 为 p 和 ui 腾出位置
					s_mc(2, h, V1, v1_t, U1 + at(h - 1, 0, u1_t), u1_t);
					// 移动 p 到新的位置
					s_mc(1, n, U + at(n - 2, 0, u_t), u_t, V + at(1, 0, v_t), v_t);

					auto q2_012_sz = std::min(n - h, Size(3));

					auto ui = V;
					auto ui_t = T(v_t);

					// 计算U2所产生的U（不经过householder变换）
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 0; j < n; ++j) {
								auto uij = at(j, 0, ui_t);
								ui[uij] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, ui, ui_t);

							auto uii = at(i, 0, ui_t);
							ui[uii] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[at(0, j, mu_t)] == -2) {
							}
							else if (mu[at(0, j, mu_t)] == -1) {
								// type 3
								auto ui0 = at(0, 0, ui_t);
								auto uij = at(j, 0, ui_t);

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[ui0] - std::sin(z[at(0, j, z_t)]) * ui[uij];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[ui0] + std::cos(z[at(0, j, z_t)]) * ui[uij];
								ui[ui0] = uia;
								ui[uij] = uib;
							}
							else {
								// type 4
								auto k = static_cast<Size>(mu[at(0, j, mu_t)]);
								auto uik = at(k, 0, ui_t);
								auto uij = at(j, 0, ui_t);

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[uik] - std::sin(z[at(0, j, z_t)]) * ui[uij];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[uik] + std::cos(z[at(0, j, z_t)]) * ui[uij];
								ui[uik] = uia;
								ui[uij] = uib;
							}
						}

						// apply permutation
						s_permutate_inv(n, 1, RowIterator<decltype(v_t)>(V + at(1, 0, v_t), v_t), ui, ui_t);

						if (i < mn) {
							s_nm(n, 1, 1.0 / s_norm(n, ui, ui_t), ui, ui_t);
						}

						// 生成 U，基于U2 //
						if (m > n) {
							auto ui0 = n > 6 ? at(0, 0, ui_t) : 0;
							s_mc(q2_012_sz, 1, s0 * ui[ui0], pre_out.q2_01.data(), 1, U + at(h + 1, i, u_t), u_t);
							s_mc(n - h - q2_012_sz, 1, s0 * ui[ui0], q2, u2_t, U + at(h + 1 + q2_012_sz, i, u_t), u_t);
							
							if (n > 6)
								s_mma(n - h, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + at(h + 1, 0, ui_t), ui_t, U + at(h + 1, i, u_t), u_t);
							else
								s_mma(n - h, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + h + 1, 1, U + at(h + 1, i, u_t), u_t);
						}
						else {
							auto minus_1 = n - h - q2_012_sz > 0 ? 0 : 1;
							auto minus_2 = n - h - q2_012_sz > 0 ? 1 : 0;

							auto ui0 = at(0, 0, ui_t);
							s_mc(q2_012_sz - minus_1, 1, s0 * ui[ui0], pre_out.q2_01.data(), 1, U + at(h + 1, i, u_t), u_t);
							s_mc(n - h - q2_012_sz - minus_2, 1, s0 * ui[ui0], q2, u2_t, U + at(h + 1 + q2_012_sz, i, u_t), u_t);

							s_mma(n - h - 1, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + at(h + 1, 0, ui_t), ui_t, U + at(h + 1, i, u_t), u_t);
						}
					}

					// 计算q2所产生的q //
					if (m > n) {
						s_mc(q2_012_sz, 1, c0, pre_out.q2_01.data(), 1, q + at(h + 1, 0, u_t), u_t);
						s_mc(n - h - q2_012_sz, 1, c0, q2, u2_t, q + at(h + 1 + q2_012_sz, 0, u_t), u_t);
					}
				}

				// 处理 m > n + 1时额外的方阵
				if (m > n + 1) {
					s_fill(m - n - 1, n + 1, 0.0, U + at(n + 1, 0, u_t), u_t);
					s_fill(n + 1, m - n - 1, 0.0, U + at(0, n + 1, u_t), u_t);
					s_eye(m - n - 1, U + at(n + 1, n + 1, u_t), u_t);
				}

				// 对U进行反 householder 变换 //
				s_householder_u_q_dot(m - 1, n, m, S + at(1, 0, s_t), s_t, U + at(1, 0, u_t), u_t, U + at(1, 0, u_t), u_t);

				// 复原压缩的V1，并移动 p 和 mu //
				// 为mu和p开辟内存
				s_mc(3, 4, S + at(n - 3, n - 4, s_t), s_t, r_of_right_corner, 4);
				// 为之后复原V1&V2开辟内存
				s_mc(3, 3, S + at(n - 2 - h, h - 3, s_t), s_t, r_of_tem_memory_for_v1v2, 3);
				// 移动p
				s_mc(1, n, V + at(1, 0, v_t), v_t, S + at(n - 2, 0, s_t), s_t);
				// 复原 V1
				s_mc(2, h, U1 + at(h - 1, 0, u1_t), u1_t, V1, v1_t);
				// 移动 mu 到新的位置
				s_mc(1, n, V + at(n - 1, 0, v_t), v_t, S + at(n - 1, 0, s_t), s_t);
			}

			// STEP 2: 计算V
			{
				auto mu = S + at(n - 1, 0, s_t);
				auto mu_t = s_t;
				
				// 根据 V1 计算
				{
					// 因为前面压缩了一行，所以这里复原 V1 到新的内存位置 //
					double V1_data_when_n_less_8_[9];

					auto V1_new = S + at(n - 2 - h, 0, s_t);
					auto v1_t_new = s_t;
					
					if (h == 1) {
						V1_data_when_n_less_8_[at(0, 0, 1)] = 1.0;
					}
					else if (h == 2) {
						s_mc(1, 2, V1, v1_t, V1_data_when_n_less_8_, 2);
						V1_data_when_n_less_8_[at(1, 0, 2)] = -V1_data_when_n_less_8_[at(0, 1, 2)];
						V1_data_when_n_less_8_[at(1, 1, 2)] = V1_data_when_n_less_8_[at(0, 0, 2)];
					}
					else if (h == 3) {

						V1_new = V1_data_when_n_less_8_;
						auto v1_t_new = 3;

						s_mc(h / 2, h, V1, v1_t, V1_new + at(0, 0, v1_t_new), v1_t_new);
						s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, V1_new + at(h / 2 + 1, 0, v1_t_new), v1_t_new);
						s_fill(1, pre_out.mn1, -1.0, V1_new + at(h / 2, 0, v1_t_new), v1_t_new);
						s_fill(1, h - pre_out.mn1, 0.0, V1_new + at(h / 2, pre_out.mn1, v1_t_new), v1_t_new);
						if (pre_out.mn1 == 0)V1_new[at(h / 2, 0, v1_t_new)] = 1.0;

						for (Size i = -1; ++i < h;) {
							s_nm(h, 1, 1.0 / s_norm(h, V1_new + at(0, i, v1_t_new), v1_t_new), V1_new + at(0, i, v1_t_new), v1_t_new);
						}
					}
					else {
						s_mc(h / 2, h, V1, v1_t, V1_new + at(0, 0, v1_t_new), v1_t_new);
						s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, V1_new + at(h / 2 + 1, 0, v1_t_new), v1_t_new);
						s_fill(1, pre_out.mn1, -1.0, V1_new + at(h / 2, 0, v1_t_new), v1_t_new);
						s_fill(1, h - pre_out.mn1, 0.0, V1_new + at(h / 2, pre_out.mn1, v1_t_new), v1_t_new);
						if (pre_out.mn1 == 0)V1_new[at(h / 2, 0, v1_t_new)] = 1.0;

						for (Size i = -1; ++i < h;) {
							s_nm(h, 1, 1.0 / s_norm(h, V1_new + at(0, i, v1_t_new), v1_t_new), V1_new + at(0, i, v1_t_new), v1_t_new);
						}
					}

					auto vi = V + at(h, 0, v_t);
					auto vi_t = T(v_t);
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 1; j < n; ++j) {
								vi[at(j, 0, vi_t)] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, vi, vi_t);
							vi[at(i, 0, vi_t)] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[at(0, j, mu_t)] == -2) {
							}
							else if (mu[at(0, j, mu_t)] == -1) {
								// type 3
							}
							else {
								// type 4
								auto via = std::cos(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] - std::sin(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
								auto vib = std::sin(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] + std::cos(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
								vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] = via;
								vi[at(j, 0, vi_t)] = vib;
							}
						}

						// apply permutation
						auto p = S + at(n - 2, 0, s_t);
						auto p_t = s_t;
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), vi, vi_t);

						if (i < mn) {
							vi[at(0, 0, vi_t)] = -1.0;
							//s_nm(n, 1, 1.0 / s_norm(n, vi, vi_t), vi, vi_t);
						}

						// 左乘V1
						V1 = V1_new;
						auto v1_t2 = v1_t_new;

						s_mm(h, 1, h, V1, v1_t2, vi + at(1, 0, vi_t), vi_t, V + at(0, i, v_t), v_t);
					}
				}

				// 根据 V2 计算
				{
					double V1_data_when_n_less_8_[9];
					
					auto V2_new = S + at(n - 2 - h, 0, s_t);
					auto v2_t_new = s_t;

					// 因为前面压缩了一行，所以这里复原 V2 到新的内存位置 //
					if (n - h - 1 == 1) {
						V1_data_when_n_less_8_[at(0, 0, 1)] = 1.0;
					}
					else if (n - h - 1 == 2) {
						s_mc(1, 2, V2, v2_t, V1_data_when_n_less_8_ + at(0, 0, 2), 2);
						V1_data_when_n_less_8_[at(1, 0, 2)] = -V1_data_when_n_less_8_[at(0, 1, 2)];
						V1_data_when_n_less_8_[at(1, 1, 2)] = V1_data_when_n_less_8_[at(0, 0, 2)];
					}
					else {
						auto k = n - h - 1;
						s_mc(k / 2, k, V2, v2_t, V2_new + at(0, 0, v2_t_new), v2_t_new);
						s_mc(k - k / 2 - 1, k, V2 + at(k / 2, 0, v2_t), v2_t, V2_new + at(k / 2 + 1, 0, v2_t_new), v2_t_new);
						s_fill(1, pre_out.mn2, -1.0, V2_new + at(k / 2, 0, v2_t_new), v2_t_new);
						s_fill(1, k - pre_out.mn2, 0.0, V2_new + at(k / 2, pre_out.mn2, v2_t_new), v2_t_new);
						if (pre_out.mn2 == 0)V2_new[at(k / 2, pre_out.mn2, v2_t_new)] = 1.0;

						for (Size i = -1; ++i < k;) {
							s_nm(k, 1, 1.0 / s_norm(k, V2_new + at(0, i, v2_t_new), v2_t_new), V2_new + at(0, i, v2_t_new), v2_t_new);
						}
					}

					auto vi = V + at(h, 0, v_t);
					auto vi_t = T(v_t);
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 1; j < n; ++j) {
								vi[at(j, 0, vi_t)] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, vi, vi_t);
							vi[at(i, 0, vi_t)] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[at(0, j, mu_t)] == -2) {
							}
							else if (mu[at(0, j, mu_t)] == -1) {
								// type 3
							}
							else {
								// type 4
								auto via = std::cos(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] - std::sin(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
								auto vib = std::sin(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] + std::cos(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
								vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] = via;
								vi[at(j, 0, vi_t)] = vib;
							}
						}

						// apply permutation
						auto p = S + at(n - 2, 0, s_t);
						auto p_t = s_t;
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), vi, vi_t);

						if (i < mn) {
							vi[at(0, 0, vi_t)] = -1.0;
						}

						// 左乘V2
						V2 = V2_new;
						auto v2_t2 = v2_t_new;
						s_mm(n - h - 1, 1, n - h - 1, V2, v2_t2, vi + at(h + 1, 0, vi_t), vi_t, V + at(h + 1, i, v_t), v_t);
					}
				}

				// 计算第h行
				for (Size i = 0; i < n; ++i) {
					V[at(h, i, v_t)] = i < mn ? -1.0 : (i == 0 ? 1.0 : 0.0);
					s_nm(n, 1, 1.0 / s_norm(n, V + at(0, i, v_t), v_t), V + at(0, i, v_t), v_t);
				}

				// 复原householders //
				double r_of_right_corner_2[12];
				s_mc(3, 4, S + at(n - 3, n - 4, s_t), s_t, r_of_right_corner_2, 4);
					
				s_mc(3, 4, r_of_right_corner, 4, S + at(n - 3, n - 4, s_t), s_t);
				s_mc(3, 3, r_of_tem_memory_for_v1v2, 3, S + at(n - 2 - h, h - 3, s_t), s_t);


				// 反 householder 变换 //
				s_householder_u_q_dot(2, 1, n, householder_vec_n, 1, V + at(n - 2, 0, v_t), v_t, V + at(n - 2, 0, v_t), v_t);
				s_householder_u_q_dot(n, n - 2, n, S + at(2, 0, s_t), T(s_t), V, v_t, V, v_t);

				// 复原 mu //
				s_mc(3, 4, r_of_right_corner_2, 4, S + at(n - 3, n - 4, s_t), s_t);
			}

			// STEP 3：计算S
			auto mu = S + at(n - 1, 0, s_t);
			auto mu_t = s_t;

			for (Size i = 0; i < n; ++i) {
				if (i < mn) {
					auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];
					S[at(0, i, s_t)] = base + mu[at(0, i, mu_t)];
				}
				else {
					S[at(0, i, s_t)] = d[at(0, i, d_t)];
				}
			}

			s_fill(m - 1, n, 0.0, S + at(1, 0, s_t), s_t);
			for (Size i = 0; ++i < n;) {
				S[at(i, i, s_t)] = S[at(0, i, s_t)];
				S[at(0, i, s_t)] = 0.0;
			}
		}

#ifdef ARIS_DEBUG_DYNAMIC_SVD
		// should check result
#endif
	}
	*/
	//auto inline s_svd(Size m, Size n, Size rank, const double *U, const double *tau, const Size *p, double *x, double *tau2, double zero_check = 1e-10)noexcept->void { s_svd(m, n, rank, U, n, tau, 1, p, x, m, tau2, 1, zero_check); }

	template<typename AType, typename UType, typename SType, typename VType>
	auto inline s_svd(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *S, SType s_t, double *V, VType v_t, double zero_check = 1e-10)noexcept->void {
		////////////////////////////////////////////// PART 1: householder变换 /////////////////////////////////
		// 对A做两次 householder 变换：
		// 其中 [ r ] 是第一次的 householder 向量
		// 此外 [ l ] 是第二次的 householder 向量
		//
		// CASE 1: 当 m = n 时
		//
		// 此时A变为：
		// [ *  r1 r1 ...  r1   r1   r1  ]
		// | *  *  r2 ...  r2   r2   r2  |
		// | l1 *  *  ...  r3   r3   r3  |
		// | l1 l2 *  ...           ..   |
		// | ..       ...  *   rn-2 rn-2 |
		// | .. ..    ...  *    *   rn-1 |
		// [ l1 l2    ... ln-2  *    *   ]
		//
		// 以上共计 n-1 次变换1，n-2次变换2
		//
		// 最终将其变成：
		//
		// [ *  *  *  ... *    ]
		// | *  *  *  ... 0    |
		// | l1 r1 r1 ... r1   |
		// | .. l2 r2 ... r2   |
		// | .. ..    ...      |
		// [ l1 l2    ... rn-2 ]
		// 同时还需额外一个变量存储rn-1
		//
		// CASE 2: 当 m = n + 1 时
		//
		// 此时A变为：
		// [ *  r1 r1 ...  r1   r1   r1  ]
		// | *  *  r2 ...  r2   r2   r2  |
		// | l1 *  *  ...  r3   r3   r3  |
		// | l1 l2 *  ...           ..   |
		// | ..       ...  *   rn-2 rn-2 |
		// | .. ..    ...  *    *   rn-1 |
		// | l1 l2    ... ln-2  *    *   |
		// [ l1 l2    ... ln-2 ln-1  *   ]
		//
		// 以上共计 n-1 次变换1，n-1次变换2
		//
		// 最终将其变成：
		//
		// [ *  *  *  ... *    ]
		// | *  *  *  ... *    |
		// | l1 r1 r1 ... r1   |
		// | .. l2 r2 ... r2   |
		// | .. ..    ...      |
		// [ l1 l2    ... rn-1 ]
		//
		// CASE 3: 当 m > n + 1 时
		//
		// 此时A变为：
		// [ *  r1 r1 ...  r1   r1   r1  ]
		// | *  *  r2 ...  r2   r2   r2  |
		// | l1 *  *  ...  r3   r3   r3  |
		// | l1 l2 *  ...           ..   |
		// | ..       ...  *   rn-2 rn-2 |
		// | .. ..    ...  *    *   rn-1 |
		// | l1 l2    ... ln-2  *    *   |
		// | l1 l2    ... ln-2 ln-1  *   |
		// [ l1 l2    ... ln-2 ln-1  ln  ]
		//
		// 以上共计 n-1 次变换1，n次变换2
		//
		// 最终将其变成：
		//
		// [ *  *  *  ... *    ]
		// | *  *  *  ... *    |
		// | l1 r1 r1 ... r1   |
		// | .. l2 r2 ... r2   |
		// | .. ..    ...      |
		// | l1 l2    ... rn-1 |
		// [ l1 l2    ... ln   ]

		////////////////////////////////////////////// PART 2: devide and conquer /////////////////////////////////
		// 迭代
		// [U S V] = dvc(A)
		// S&A: n+1 x n
		// U  : n+1 x n+1
		// V  :   n x n
		// S 和 A 位于同一片内存
		//
		// A 为 双对角矩阵：
		// [ *         ]
		// | * *       |
		// |   * *     |
		// |     * *   |
		// |       * * |
		// [         * ]
		//
		// 但是在内存中储存为，记做：
		// [ * *  ... * ]
		// | * *  ... * |
		// |            |
		// [            ]
		//
		//  STEP 1：递归调用
		//  h = n/2, n为偶数；或 (n-1)/2, n为奇数
		// 	[U1 S1 V1] = dvc(A(   1:h+1 , 1:h   ));
		//  [U2 S2 V2] = dvc(A( h+2:n+1 , h+2:n ));
		//
		//  各矩阵维度：  U1:h+1 x h+1    S1:h+1 x h        V1:    h x h
		//                U2:n-h x n-h    S2:n-h x n-h-1    V2:n-h-1 x n-h-1
		//
		//  【【注意】】：以上求值顺序不可改变，因后续计算中，需要压缩 V 来省内存空间，因此U2，V2的计算会覆盖V1的最后一行。
		//  【【注意】】：所有返回的V，为了存储紧凑，均存成n-1 x n 维，因为vi中的首个元素均为-1
		//
		//  调用前的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ EMPTY ]
		//	--------------------------------------------
		//	S：
		//	      h   1   n-h-1
		//	2   [ A1  ek  A2   ]
		//	n-1 [ EMPTY....... ]
		//	--------------------------------------------
		//	V:
		//	[ empty ]
		//	--------------------------------------------
		//
		//
		//  调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ EMPTY ]
		//	--------------------------------------------
		//	S：
		//  1  [ d1       d2 ]
		//  2  | ...empty... |
		// n+1 [ ...empty... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)，此时用额外的3维向量存储 q1 的突出部
		//           1      h   h+1    h+2    n
		//                      q1
		//                      q1
		//      1  [ u1 ... u1  q1     v1 ... v1 ]
		//         | .  ... .   .      v1 ... v1 |
		//         | u1 ... u1  q1     v1 ... v1 |
		//      h  | u1  ...u1  u2     u2 ... q2 |
		//     h+1 | u1 ... u1  u2     u2 ... q2 |
		//     h+2 | v2 ... v2  .      .  ... .  |
		//         | .  ... .   .      .  ... .  |
		//     n-1 | v2 ... v2  u2     .  ... q2 |
		//      n  [ ......   empty  ........    ]
		//
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h      h+1    h+2    n-1 n
		//      1  [ u1 ... u1   u1     v1     v1 ... v1  v1]
		//         | .  ... .    .      .      .  ... .   . |
		//     h-1 | .  ... .    u1&q1  v1     v1 ... v1  v1|
		//      h  | .  ... .    u1&q1  u2     u2 ... u2  q2|
		//     h+1 | u1 ... u1   u1&q1  u2     u2 ... u2  q2|
		//     h+2 | v2 ... v2   q1     u2     .  ... .   . |
		//         | .  ... .    .      .      .  ... .   . |
		//     n-1 [ v2 ... v2   q1     u2     u2 ... u2  q2]
		//      n  [ .................empty.................]
		//
		// STEP 2：构造  d  z  mu  p
		//
		// 其中：
		// d 和 z 构成了 M：
		// [ z1            ]
		// | z2  d2        |
		// | ...    ...    |
		// [ zn         dn ]
		//
		// 以上内存分别存储于不同的地方，因此调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ EMPTY ]
		//	--------------------------------------------
		//	S：
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  p  ... |
		// n+1 [ ...empty... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)，此时用额外的3维向量存储 q1 的突出部
		//
		//           1      h   h+1    h+2    n
		//                      q1
		//                      q1
		//      1  [ u1 ... u1  q1     v1 ... v1 ]
		//         | .  ... .   .      v1 ... v1 |
		//         | u1 ... u1  q1     v1 ... v1 |
		//      h  | u1  ...u1  u2     u2 ... q2 |
		//     h+1 | u1 ... u1  u2     u2 ... q2 |
		//     h+2 | v2 ... v2  .      .  ... .  |
		//         | .  ... .   .      .  ... .  |
		//     n-1 | v2 ... v2  u2     .  ... q2 |
		//      n  [ .............mu.............]
		//
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h      h+1    h+2    n-1 n
		//      1  [ u1 ... u1   u1     v1     v1 ... v1  v1]
		//         | .  ... .    .      .      .  ... .   . |
		//     h-1 | .  ... .    u1&q1  v1     v1 ... v1  v1|
		//      h  | .  ... .    u1&q1  u2     u2 ... u2  q2|
		//     h+1 | u1 ... u1   u1&q1  u2     u2 ... u2  q2|
		//     h+2 | v2 ... v2   q1     u2     .  ... .   . |
		//         | .  ... .    .      .      .  ... .   . |
		//     n-1 [ v2 ... v2   q1     u2     u2 ... u2  q2]
		//      n  [ ...................mu..................]
		//
		// STEP 3：构造 U
		//
		// 在该步中，ui存于S的第4行处(因 n+1>=4)
		//
		//  调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ full of U ]
		//	--------------------------------------------
		//	S：
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  p  ... |
		//  4  | ...  ui ... |
		// n+1 [ ...empty... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)，此时用额外的3维向量存储 q1 的突出部
		//
		//           1      h   h+1  h+2    n
		//      1  [ .....empty..... v1 ... v1 ]
		//         | .....empty..... v1 ... v1 |
		//         | .....empty..... v1 ... v1 |
		//      h  | .....empty.........empty..|
		//     h+1 | .....empty.........empty..|
		//     h+2 | v2 ... v2 .........empty..|
		//         | .  ... .  .........empty..|
		//     n-1 | v2 ... v2 .........empty..|
		//      n  [ ..........mu..............]
		//
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h   h+1 h+2    n-1 n
		//      1  [ .....empty..... v1  v1 ... v1  v1]
		//         | .....empty..... .   .  ... .   . |
		//     h-1 | .....empty..... v1  v1 ... v1  v1|
		//      h  | .....empty..........empty........|
		//     h+1 | .....empty..........empty........|
		//     h+2 | v2 ... v2 ..........empty........|
		//         | .  ... .  ..........empty........|
		//     n-1 | v2 ... v2 ..........empty........|
		//      n  [ ...............mu................]
		//
		//
		//
		// STEP 4：构造 V
		//
		// 在该步中，vi可能存于p处，或S的最后一行处
		//
		// vi位置：
		//	--------------------------------------------
		//	S：
		//    n >= 5 时：
		//
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  p  ... |
		// 4.. | V1       V2 |
		// n+1 [ ...  vi ... ]
		//    n < 5 时：(p存于其他地方)
		//
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  vi ... |
		// 4.. | V1       V2 |
		//	--------------------------------------------
		//
		//
		//  调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ full of U ]
		//	--------------------------------------------
		//	S：
		//
		//  1  [ ...  d  ... ]
		//  2  | ...  z  ... |
		//  3  | ...  p  ... |
		//  4  | ...  ui ... |
		// n+1 [ ...empty... ]
		//	--------------------------------------------
		//
		//	V：
		//   1:n-1 [ compressed V ]
		//      n  [ ....mu.......]
		//
		// STEP 5：构造 S
		//
		//  调用后的内存布局：
		//  --------------------------------------------
		//	U：
		//	[ full of U ]
		//	--------------------------------------------
		//	S：
		//
		//  1     [ ...  d  ... ]
		//  2:n+1 [ ...empty... ]
		//	--------------------------------------------
		//
		//	V：
		//   1:n-1 [ compressed V ]
		//      n  [ ...empty.....]

		////////////////////////////////////////////// PART 3: 首次调用 /////////////////////////////////
		//  首次经过dvc调用后，整体的内存布局如下：
		//  --------------------------------------------
		//	U：
		//	1:n-1 [ EMPTY ]
		//  n     [...p...]
		//  n+1:m [ EMPTY ]
		//	--------------------------------------------
		//	S：
		//	1   [ ......d....... ]
		//	2   | ......z....... |
		//	n-1 [ househoders... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)，此时用额外的3维向量存储 q1 的突出部
		//           1      h   h+1    h+2    n
		//                      q1
		//                      q1
		//      1  [ u1 ... u1  q1     v1 ... v1 ]
		//         | .  ... .   .      v1 ... v1 |
		//         | u1 ... u1  q1     v1 ... v1 |
		//      h  | u1  ...u1  u2     u2 ... q2 |
		//     h+1 | u1 ... u1  u2     u2 ... q2 |
		//     h+2 | v2 ... v2  .      .  ... .  |
		//         | .  ... .   .      .  ... .  |
		//     n-1 | v2 ... v2  u2     .  ... q2 |
		//      n  [ .........mu.................]
		//
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h      h+1    h+2    n-1 n
		//      1  [ u1 ... u1   u1     v1     v1 ... v1  v1 ]
		//         | .  ... .    .      .      .  ... .   .  |
		//     h-1 | .  ... .    u1&q1  v1     v1 ... v1  v1 |
		//      h  | .  ... .    u1&q1  u2     u2 ... u2  q2 |
		//     h+1 | u1 ... u1   u1&q1  u2     u2 ... u2  q2 |
		//     h+2 | v2 ... v2   q1     u2     .  ... .   .  |
		//         | .  ... .    .      .      .  ... .   .  |
		//     n-1 | v2 ... v2   q1     u2     u2 ... u2  q2 |
		//      n  [ ..................mu....................]
		//
		//  STEP 1:计算U
		//  若 n > 6 时，先计算U1所产生的 U（不做householder变换），此后将此时内存变化如下：
		//  --------------------------------------------
		//	U：
		//	1    [ u ... u ]
		//  ...  |         |
		//  h+1  | u ... u |
		//  ...  | .empty. |
		//  n    | .empty. |
		//  ...  | .empty. |
		//  m    [ .empty. ]
		//	--------------------------------------------
		//	S：
		//	1   [ ......d....... ]
		//	2   | ......z....... |
		//	n-1 [ househoders... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：  此时 mU1 == mU2 (h+1 == n-h)
		//           1      h   h+1    h+2    n
		//      1  [ ...........ui.............. ]
		//      2  | ...........p............... |
		//         | ....EMPTY....     v1 ... v1 |
		//      h  | v1 ... v1  u2     u2 ... q2 |
		//     h+1 | v1 ... v1  u2     u2 ... q2 |
		//     h+2 | v2 ... v2  .      .  ... .  |
		//         | .  ... .   .      .  ... .  |
		//     n-1 | v2 ... v2  u2     .  ... q2 |
		//      n  [ .........mu.................]
		//
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h   h+1    h+2    n-1 n
		//      1  [ ...........ui....................... ]
		//      2  | ...........p........................ |
		//         | ....EMPTY....   v1     v1 ... v1  v1 |
		//     h-1 | ....EMPTY....   v1     v1 ... v1  v1 |
		//      h  | v1 ... v1   v1  u2     u2 ... u2  q2 |
		//     h+1 | v1 ... v1   v1  u2     u2 ... u2  q2 |
		//     h+2 | v2 ... v2       u2     .  ... .   .  |
		//         | .  ... .        .      .  ... .   .  |
		//     n-1 | v2 ... v2       u2     u2 ... u2  q2 |
		//      n  [ ..................mu.................]
		//
		//  再计算U2产生的矩阵，并进行反householder变换，结果如下：
		//
		//  --------------------------------------------
		//	U：
		//	    [ full of U ]
		//	--------------------------------------------
		//	S：
		//	1   [ .......d....... ]
		//	2   | .......z....... |
		//	3   | *  r  ......... |
		//	4   | *  *  r  ...... |
		//	... | .......p....... |
		//	n   | ......mu....... |    // 右下角用r_corner 来临时保存，为3 x 4 的矩阵
		//  ... [ ............... ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：
		//           1      h   h+1    h+2    n
		//      1  [ .....EMPTY.....   v1 ... v1 ]
		//         | .....EMPTY.....   v1 ... v1 |
		//         | .....EMPTY.....   v1 ... v1 |
		//      h  | .....EMPTY..........EMPTY.. |
		//     h+1 | .....EMPTY..........EMPTY.. |
		//     h+2 | v2 ... v2  .........EMPTY.. |
		//         | .  ... .   .........EMPTY.. |
		//     n-1 | v2 ... v2  .........EMPTY.. |
		//      n  [ .....EMPTY..........EMPTY.. ]
		//
		//    n 为偶数时：  此时 mU1 == mU2+1 (h+1 == n-h + 1)
		//           1      h-1  h      h+1    h+2    n-1 n
		//      1  [ .....EMPTY.....    v1     v1 ... v1  v1 ]
		//         | .....EMPTY.....    .      .  ... .   .  |
		//     h-1 | .....EMPTY.....    v1     v1 ... v1  v1 |
		//      h  | .....EMPTY..............EMPTY.......... |
		//     h+1 | .....EMPTY..............EMPTY.......... |
		//     h+2 | v2 ... v2   ............EMPTY.......... |
		//         | .  ... .    ............EMPTY.......... |
		//     n-1 | v2 ... v2   ............EMPTY.......... |
		//      n  [ .....EMPTY..............EMPTY.......... ]
		//
		//
		//  STEP 2:计算V
		//  若 n > 6 时，先计算V1所产生的 V（不做householder变换），此后将此时内存变化如下：
		//  --------------------------------------------
		//	U：
		//	    [ full of U ]
		//	--------------------------------------------
		//	S：
		//          1  2  3 ... h
		//	1     [ .......d.......... ]
		//	2     | .......z.......... |
		//	3     | *  r  ............ |
		//	4     | *  *  r  ......... |
		//  ...   |                    |
		//  n-1-h | v1 ........ v1 r   |    // 右上方需要保存起点为 n-1-h, h-3 的 3x3 的矩阵
		//  ...   |                    |
		//  n-2   | v1 ........ v1 r   |
		//	n-1   | .......p.......... |
		//	n     | ......mu.......... |    // 右下角用r_corner 来临时保存，为3 x 4 的矩阵
		//  ...   [ .................. ]
		//	--------------------------------------------
		//	V：
		//    n 为奇数时：
		//           1    n-h-1              n
		//      1  [ .....EMPTY..........EMPTY.. ]
		//         | .....EMPTY..........EMPTY.. |
		//     h+1 | .....EMPTY..........EMPTY.. |
		//     h+2 | v2 ... v2  .........EMPTY.. |
		//         | .  ... .   .........EMPTY.. |
		//     n-1 | v2 ... v2  .........EMPTY.. |
		//      n  [ .....EMPTY..........EMPTY.. ]
		//
		//
		//

		// check size //
		if (m == 0 || n == 0) return;

		// check m > n
		if (n > m) {
			s_svd(n, m, A, T(a_t), V, v_t, S, T(s_t), U, u_t);
			return;
		}

		// n == 1 //
		if (n == 1) {
			s_mc(m, n, A, a_t, S, s_t);

			if (m == 1) {
				U[at(0, 0, u_t)] = 1;
				V[at(0, 0, v_t)] = s_sgn2(S[at(0, 0, s_t)]);
				S[at(0, 0, s_t)] = s_sgn2(S[at(0, 0, s_t)]) * S[at(0, 0, s_t)];
			}
			else {
				// householder 变换 //
				double rho = -s_sgn2(S[at(0, 0, s_t)]) * std::sqrt(s_vv(m, S + at(0, 0, s_t), s_t, S + at(0, 0, s_t), s_t));
				s_nm(m - 1, 1, std::abs(rho) > zero_check ? 1.0 / (S[at(0, 0, s_t)] - rho) : 0.0, S + at(1, 0, s_t), s_t);

				// U //
				s_eye(m, U, u_t);
				s_householder_u_q_dot(m, n, m, S + at(0, 0, s_t), s_t, U + at(0, 0, u_t), u_t, U + at(0, 0, u_t), u_t);

				// V //
				V[at(0, 0, v_t)] = s_sgn2(rho);

				// S //
				s_fill(m, n, 0.0, S, s_t);
				S[at(0, 0, s_t)] = s_sgn2(rho) * rho;
			}
			return;
		}

		// n == 2 //
		if (n == 2) {
			s_mc(m, n, A, a_t, S, s_t);

			double tau[2];
			s_householder_ut(m, n, S, s_t, S, s_t, tau, 1);

			auto a = S[at(0, 0, s_t)];
			auto b = S[at(0, 1, s_t)];
			auto d = S[at(1, 1, s_t)];

			auto theta3 = 0.5 * std::atan2(2 * b * d, a * a + b * b - d * d);
			auto c3 = std::cos(theta3);
			auto s3 = std::sin(theta3);

			auto S1 = a * a + b * b + d * d;
			auto S2 = std::sqrt((a * a + b * b - d * d) * (a * a + b * b - d * d) + 4 * b * b * d * d);

			auto phi = 0.5 * std::atan2(2 * a * b, a * a - b * b - d * d);
			auto c_p = std::cos(phi);
			auto s_p = std::sin(phi);
			auto s11 = (a * c3) * c_p + (b * c3 + d * s3) * s_p;
			auto s22 = (a * s3) * s_p + (-b * s3 + d * c3) * c_p;

			// U //
			s_eye(m, U, u_t);
			U[at(0, 0, u_t)] = s_sgn2(s11) * (c3);
			U[at(0, 1, u_t)] = s_sgn2(s22) * (-s3);
			U[at(1, 0, u_t)] = s_sgn2(s11) * (s3);
			U[at(1, 1, u_t)] = s_sgn2(s22) * (c3);

			// 对U进行反 householder 变换 //
			s_householder_ut_q_dot(m, n, m, S + at(0, 0, s_t), s_t, tau, 1, U + at(0, 0, u_t), u_t, U + at(0, 0, u_t), u_t);

			// V //
			V[at(0, 0, v_t)] = c_p;
			V[at(0, 1, v_t)] = -s_p;
			V[at(1, 0, v_t)] = s_p;
			V[at(1, 1, v_t)] = c_p;

			// S //
			s_fill(m, n, 0.0, S, s_t);
			S[at(0, 0, s_t)] = std::sqrt((S1 + S2) / 2);
			S[at(1, 1, s_t)] = std::sqrt((S1 - S2) / 2);

			return;
		}

		//////////////////////////////////////////////////////////// PART 1: householder 变换 /////////////////////////////////
		s_mc(m, n, A, a_t, S, s_t);
		for (Size i = 0; i < n; ++i) {
			// compute householder vector 1 //
			if (i < n - 1){
				double rho = -s_sgn2(S[at(i, i, s_t)]) * std::sqrt(s_vv(n - i, S + at(i, i, s_t), T(s_t), S + at(i, i, s_t), T(s_t)));
				double tau_reciprocal = std::abs(rho) > zero_check ? S[at(i, i, s_t)] / rho - 1.0 : -2.0;   // tau 的倒数，tau和论文中定义一致
				s_nm(1, n - 1 - i, std::abs(rho) > zero_check ? 1.0 / (S[at(i, i, s_t)] - rho) : 0.0, S + at(i, i + 1, s_t), s_t);

				// update matrix 1 //
				for (Size j(i); ++j < m;) {
					double k = tau_reciprocal * (s_vv(n - i - 1, S + at(i, i + 1, s_t), T(s_t), S + at(j, i + 1, s_t), T(s_t)) + S[at(j, i, s_t)]);
					S[at(j, i, s_t)] += k;
					s_ma(1, n - i - 1, k, S + at(i, i + 1, s_t), s_t, S + at(j, i + 1, s_t), s_t);
				}
				S[at(i, i, s_t)] = rho;
			}

			// compute householder vector 2, CASE 1 2 3 //
			if ((n > 1 && i < n - 2) || (m == n + 1 && i < n - 1) || (m > n + 1 && i < n)) {
				double rho = -s_sgn2(S[at(i + 1, i, s_t)]) * std::sqrt(s_vv(m - i - 1, S + at(i + 1, i, s_t), s_t, S + at(i + 1, i, s_t), s_t));
				double tau_reciprocal = std::abs(rho) > zero_check ? S[at(i + 1, i, s_t)] / rho - 1.0 : -2.0;   // tau 的倒数，tau和论文中定义一致
				s_nm(m - 2 - i, 1, std::abs(rho) > zero_check ? 1.0 / (S[at(i + 1, i, s_t)] - rho) : 0.0, S + at(i + 2, i, s_t), s_t);

				// update matrix 2 //
				for (Size j(i); ++j < n;) {
					double k = tau_reciprocal * (s_vv(m - i - 2, S + at(i + 2, i, s_t), s_t, S + at(i + 2, j, s_t), s_t) + S[at(i + 1, j, s_t)]);
					S[at(i + 1, j, s_t)] += k;
					s_va(m - i - 2, k, S + at(i + 2, i, s_t), s_t, S + at(i + 2, j, s_t), s_t);
				}
				S[at(i + 1, i, s_t)] = rho;
			}

			// 用 U 来存储 d 和 z //
			U[at(0, i, u_t)] = S[at(i, i, s_t)];
			U[at(1, i, u_t)] = m == n && i == n - 1 ? 0.0 : S[at(i + 1, i, s_t)];
		}

		// reconstruct matrix //
		double householder_vec_n[2]{ 0.0, S[at(n - 2, n - 1, s_t)] };
		for (Size i(1); ++i < n;) {
			s_mc(1, i, S + at(n - 1 - i, n - i, s_t), s_t, S + at(n + 1 - i, n - i, s_t), s_t);
		}

		//////////////////////////////////////////////////////////// PART 2: devide and conquer ///////////////////////////////
#ifdef ARIS_DEBUG_DYNAMIC_SVD
		struct DebugStruct {
			int level;
			int num;
			Size n;
			std::vector<double> U, S, V;
		};
		std::list<DebugStruct> _D_list;
#endif
		struct DvcPreOut {
			Size mn;
			double dn;
			double q2_0, q1_e;
			double r0, c0, s0;
			Size mn1, mn2;
		};

		const auto dvc_pre = [
#ifdef ARIS_DEBUG_DYNAMIC_SVD
			&
#else
			&
#endif
		](Size n, double* S, auto s_t, double* V, auto v_t, double* q, auto q_t, const auto& dvc, const auto& dvc_pre)->DvcPreOut
		{
#ifdef ARIS_DEBUG_DYNAMIC_SVD
			DebugStruct _D_s;
			_D_s.n = n;
			_D_s.U.resize((n + 1)* (n + 1));
			_D_s.S.resize((n + 1)* (n));
			_D_s.V.resize((n)* (n));

			s_mc(n + 1, n + 1, U, u_t, _D_s.U.data(), n + 1);
			s_mc(n + 1, n,     S, s_t, _D_s.S.data(), n);
			s_mc(n,     n,	   V, v_t, _D_s.V.data(), n);

			if (_D_list.empty()) {
				_D_s.level = 0;
				_D_s.num = 0;
				_D_list.push_back(_D_s);
			}
			else {
				_D_s.num = 0;
				_D_s.level = _D_list.back().level + 1;
				_D_list.push_back(_D_s);
			}
#endif
			DvcPreOut pre_out = {};

			const Size MAX_N = 9;

			const auto make_dzpmu = [](Size n, double* S, auto s_t, double* V, auto v_t, double* q, auto q_t, double *dzp, auto dzp_t, DvcPreOut &pre_out, const auto& dvc, const auto& dvc_pre)->void {
				auto h = n / 2;
				auto U1 = V + at(0, 0, v_t);
				auto u1_t = v_t;
				auto S1 = S + at(0, 0, s_t);
				auto s1_t = s_t;
				auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
				auto v1_t = v_t;
				auto U2 = V + at(h, h, v_t);
				auto u2_t = v_t;
				auto S2 = S + at(0, h + 1, s_t);
				auto s2_t = s_t;
				auto V2 = V + at(h + 1, 0, v_t);
				auto v2_t = v_t;

				const auto a_h_h = S[at(0, h, s_t)];
				const auto a_hp1_h = S[at(1, h, s_t)];

				auto q1 = q + at(0, 0, q_t);
				auto q2 = q + at(h + 1, 0, q_t);

				auto ret1 = dvc(h, U1, u1_t, q1, q_t, S1, s1_t, V1, v1_t, dvc, dvc_pre);
				auto q1_e = q1[h];

				auto ret2 = dvc(n - h - 1, U2, u2_t, q2, q_t, S2, s2_t, V2, v2_t, dvc, dvc_pre);
				auto q2_0 = q2[0];

				// # STEP 2：构造 d、z、mu、p  //
				// ## STEP 2.1：计算排序的 d z p  //
				auto r0 = std::sqrt(a_h_h * a_h_h * q1_e * q1_e + a_hp1_h * a_hp1_h * q2_0 * q2_0);
				auto theta = std::atan2(a_hp1_h * q2_0, a_h_h * q1_e);
				auto c0 = std::cos(theta);
				auto s0 = std::sin(theta);

				auto d = S;
				auto d_t = s_t;
				auto z = S + at(1, 0, s_t);
				auto z_t = s_t;
				auto mu = S + at(2, 0, s_t);
				auto mu_t = s_t;
				auto p = S + at(3, 0, s_t);
				auto p_t = s_t;

				// 把S1的内存右移一格
				for (Size i = h; --i < h;) {
					d[at(0, i + 1, d_t)] = d[at(0, i, d_t)];
				}
				d[at(0, 0, d_t)] = 0.0;
				z[at(0, 0, z_t)] = r0;

				// 生成z //
				s_mc(1, h, a_h_h, U1 + at(h, 0, u1_t), u1_t, z + at(0, 1, z_t), z_t);
				s_mc(1, n - h - 2, a_hp1_h, U2 + at(0, 1, u2_t), u2_t, z + at(0, h + 2, z_t), z_t);
				z[at(0, h + 1, z_t)] = a_hp1_h * U2[at(0, 0, u2_t)];

				// 排序生成p
				for (auto i = 0; i < n; ++i)p[at(0, i, p_t)] = i;
				std::sort(RowIterator<decltype(p_t)>(p, p_t), RowIterator<decltype(p_t)>(p, p_t) + n, [&d, &d_t](const auto& left, const auto& right) {
					return d[at(0, static_cast<Size>(left), d_t)] < d[at(0, static_cast<Size>(right), d_t)];
					});

				// d和z两个一起变换
				s_permutate(n, 2, RowIterator<decltype(p_t)>(p, p_t), d, T(d_t));

				// ## STEP 2.2：deflation
				auto max_abs_z = std::abs(*std::max_element(RowIterator<decltype(z_t)>(z, z_t), RowIterator<decltype(z_t)>(z, z_t) + n, [](const auto& l, const auto& r) {return std::abs(l) < std::abs(r); }));
				auto max_d = d[at(0, n - 1, d_t)];
				constexpr auto consider_zero = std::numeric_limits<double>::min();
				const auto epsilon_strict = std::max(consider_zero, std::numeric_limits<double>::epsilon() * max_d);
				const auto epsilon_coarse = std::max(consider_zero, 8.0 * std::numeric_limits<double>::epsilon() * std::max(max_abs_z, max_d));

				auto mn = n;
				for (Size i = 1; i < mn; ) {
					// check zi near zero, deflation type 2
					if (std::abs(z[at(0, i, z_t)]) < epsilon_strict) {
						mu[at(0, mn - 1, mu_t)] = -2;
						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);
						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;
						--mn;

						continue;
					}
					else if (std::abs(d[at(0, i, d_t)]) < epsilon_coarse) {
						mu[at(0, mn - 1, mu_t)] = -1;
						auto z1 = std::sqrt(z[at(0, 0, z_t)] * z[at(0, 0, z_t)] + z[at(0, i, z_t)] * z[at(0, i, z_t)]);
						auto zi = std::atan2(z[at(0, i, z_t)], z[at(0, 0, z_t)]);
						z[at(0, 0, z_t)] = z1;
						z[at(0, i, z_t)] = zi;
						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);
						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;
						--mn;

						continue;
					}
					else if (d[at(0, i, d_t)] - d[at(0, i - 1, d_t)] < epsilon_strict) {
						mu[at(0, mn - 1, mu_t)] = (double)(i - 1);
						auto z_im1 = std::sqrt(z[at(0, i, z_t)] * z[at(0, i, z_t)] + z[at(0, i - 1, z_t)] * z[at(0, i - 1, z_t)]);
						auto z_i = std::atan2(z[at(0, i, z_t)], z[at(0, i - 1, z_t)]);
						z[at(0, i - 1, z_t)] = z_im1;
						z[at(0, i, z_t)] = z_i;

						auto tem = d[at(0, i, d_t)];
						s_mc(1, mn - i, d + at(0, i + 1, d_t), d_t, d + at(0, i, d_t), d_t);
						d[at(0, mn - 1, d_t)] = tem;
						tem = z[at(0, i, z_t)];
						s_mc(1, mn - i, z + at(0, i + 1, z_t), z_t, z + at(0, i, z_t), z_t);

						z[at(0, mn - 1, z_t)] = tem;
						tem = p[at(0, i, d_t)];
						s_mc(1, mn - i, p + at(0, i + 1, p_t), p_t, p + at(0, i, p_t), p_t);
						p[at(0, mn - 1, p_t)] = tem;

						--mn;
					}
					else {
						++i;
					}
				}

				auto dn = 0.0;
				if (mn == 1) {
					mu[at(0, mn - 1, mu_t)] = -2;
					mn = 0;
					d[at(0, 0, d_t)] = z[at(0, 0, z_t)];
					dn = z[at(0, 0, z_t)];
				}
				else {
					if (z[at(0, 0, z_t)] <= epsilon_coarse)
						z[at(0, 0, z_t)] = epsilon_coarse;

					dn = d[at(0, mn - 1, d_t)] + std::sqrt(s_vv(mn, z, T(z_t), z, T(z_t)));
				}

				// ## STEP 2.3：计算奇异值(mu)
				for (Size i{ 0 }, di{ 0 }; i < mn; ++i, di = next_c(di, d_t)) {
					auto left = d[di];
					auto right = i == mn - 1 ? dn : d[next_c(di, d_t)];
					auto mid = 0.5 * (left + right);
					auto v = 0.0;
					for (Size j = 0; j < mn; ++j) {
						v += z[at(0, j, z_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - mid) / (d[at(0, j, d_t)] + mid);
					}

					auto base = v < -1 ? right : left;
					auto lower = v < -1 ? left - right : 0.0;
					auto upper = v < -1 ? 0.0 : right - left;

					while (std::abs(lower - upper) > (std::max(std::abs(lower), std::abs(upper))) * 2 * std::numeric_limits<double>::epsilon()) {
						mid = (lower + upper) / 2;
						auto v = 0.0;
						for (Size j = 0; j < mn; ++j) {
							v += z[at(0, j, z_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mid) / (d[at(0, j, d_t)] + base + mid);
						}
						v < -1 ? lower = mid : upper = mid;
					}
					mu[at(0, i, mu_t)] = (lower + upper) / 2;
				}

				// ## STEP 2.4：重新计算 z
				for (Size i = 0, d_i = 0, z_i = 0; i < mn; ++i, d_i = next_c(d_i, d_t), z_i = next_c(z_i, z_t)) {
					auto base = mu[at(0, mn - 1, mu_t)] < 0.0 ? dn : d[at(0, mn - 1, d_t)];
					auto zi = (base + d[at(0, i, d_t)] + mu[at(0, mn - 1, mu_t)]) * (base - d[at(0, i, d_t)] + mu[at(0, mn - 1, mu_t)]);

					for (Size k = 0, mu_k = 0, d_k = 0, d_kp1 = next_c(d_k, d_t); k < i; ++k, mu_k = next_c(mu_k, mu_t), d_k = d_kp1, d_kp1 = next_c(d_k, d_t)) {
						base = mu[mu_k] < 0 ? d[d_kp1] : d[d_k];
						zi *= (base - d[d_i] + mu[mu_k]) * (base + d[d_i] + mu[mu_k]) / (d[d_k] - d[d_i]) / (d[d_k] + d[d_i]);
					}

					for (Size k = i, mu_k = at(0, k, mu_t), d_k = at(0, k, d_t), d_kp1 = next_c(d_k, d_t); k < mn - 1; ++k, mu_k = next_c(mu_k, mu_t), d_k = d_kp1, d_kp1 = next_c(d_k, d_t)) {
						base = mu[mu_k] < 0 ? d[d_kp1] : d[d_k];
						zi *= (base - d[d_i] + mu[mu_k]) * (base + d[d_i] + mu[mu_k]) / (d[d_kp1] - d[d_i]) / (d[d_kp1] + d[d_i]);
					}

					z[z_i] = s_sgn2(z[z_i]) * std::sqrt(zi);
				}

				pre_out = DvcPreOut{
					mn,
					dn,
					q2_0, q1_e,
					r0, c0, s0,
					ret1.mn, ret2.mn,
				};
			};

			if (n > 2 && n < MAX_N) {
				double dzp[MAX_N * 3];
				make_dzpmu(n, S, s_t, V, v_t, q, q_t, dzp, MAX_N, pre_out, dvc, dvc_pre);
			}
			else if(n > 2) {
				double dzp[MAX_N * 3];
				make_dzpmu(n, S, s_t, V, v_t, q, q_t, dzp, MAX_N, pre_out, dvc, dvc_pre);
			}

			return pre_out;
		};

		const auto dvc = [
#ifdef ARIS_DEBUG_DYNAMIC_SVD
			&
#else
			&
#endif
		](Size n, double *U, auto u_t, double*q, auto q_t, double *S, auto s_t, double *V, auto v_t, const auto&dvc, const auto& dvc_pre)->DvcPreOut
		{
			auto pre_out = dvc_pre(n, S, s_t, V, v_t, q, q_t, dvc, dvc_pre);

			const Size MAX_N = 11;

			const auto dvc_inside = [](Size n, double *U, auto u_t, double*q, auto q_t, double *S, auto s_t, double *V, auto v_t, const auto&dvc, const auto& dvc_pre, DvcPreOut &pre_out) {
				auto h = n / 2;
				auto U1 = V + at(0, 0, v_t);
				auto u1_t = v_t;
				auto S1 = S + at(0, 0, s_t);
				auto s1_t = s_t;
				auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
				auto v1_t = v_t;
				auto U2 = V + at(h, h, v_t);
				auto u2_t = v_t;
				auto S2 = S + at(0, h + 1, s_t);
				auto s2_t = s_t;
				auto V2 = V + at(h + 1, 0, v_t);
				auto v2_t = v_t;

				auto q1 = q;
				auto q2 = q + at(h + 1, 0, 1);

				auto c0 = pre_out.c0;
				auto s0 = pre_out.s0;
				auto mn = pre_out.mn;
				auto dn = pre_out.dn;

				auto d = S;
				auto d_t = s_t;
				auto z = S + at(1, 0, s_t);
				auto z_t = s_t;
				auto mu = S + at(2, 0, s_t);
				auto mu_t = s_t;
				auto p = S + at(3, 0, s_t);
				auto p_t = s_t;

				//STEP 3：计算U
				auto ui = S + at(4, 0, s_t);
				auto ui_t = T(s_t);

				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

						for (Size j = 0; j < n; ++j) {
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);
							ui[uij] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
						}
					}
					else {
						s_fill(n, 1, 0.0, ui, ui_t);
						auto uii = at(static_cast<Size>(p[at(0, i, p_t)]), 0, ui_t);
						ui[uii] = 1.0;
					}

					// apply deflation
					for (Size j = mn; j < n; ++j) {
						if (mu[at(0, j, mu_t)] == -2) {
						}
						else if (mu[at(0, j, mu_t)] == -1) {
							auto ui0 = at(static_cast<Size>(p[at(0, 0, p_t)]), 0, ui_t);
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);

							// type 3
							auto uia = std::cos(z[at(0, j, z_t)]) * ui[ui0] - std::sin(z[at(0, j, z_t)]) * ui[uij];
							auto uib = std::sin(z[at(0, j, z_t)]) * ui[ui0] + std::cos(z[at(0, j, z_t)]) * ui[uij];
							ui[ui0] = uia;
							ui[uij] = uib;
						}
						else {
							// type 4
							auto idx = static_cast<Size>(mu[at(0, j, mu_t)]);
							auto uik = at(static_cast<Size>(p[at(0, idx, p_t)]), 0, ui_t);
							auto uij = at(static_cast<Size>(p[at(0, j, p_t)]), 0, ui_t);

							auto uia = std::cos(z[at(0, j, z_t)]) * ui[uik] - std::sin(z[at(0, j, z_t)]) * ui[uij];
							auto uib = std::sin(z[at(0, j, z_t)]) * ui[uik] + std::cos(z[at(0, j, z_t)]) * ui[uij];
							ui[uik] = uia;
							ui[uij] = uib;
						}
					}

					if (i < mn) {
						s_nm(n, 1, 1.0 / s_norm(n, ui, ui_t), ui, ui_t);
					}

					// 生成 U，基于U1 //
					s_mc(h + 1, 1, c0 * ui[at(0, 0, ui_t)], q1, 1, U + at(0, i, u_t), u_t);
					s_mma(h + 1, 1, h, U1 + at(0, 0, u1_t), u1_t, ui + at(1, 0, ui_t), ui_t, U + at(0, i, u_t), u_t);

					// 生成 U，基于U2 //
					s_mc(n - h, 1, s0 * ui[at(0, 0, ui_t)], q2, 1, U + at(h + 1, i, u_t), u_t);
					s_mma(n - h, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + at(h + 1, 0, ui_t), ui_t, U + at(h + 1, i, u_t), u_t);
				}

				s_nm(h + 1, 1, -s0, q1, q_t);
				s_nm(n - h, 1, c0, q2, q_t);

				// step 7：复原V1 & V2到正确位置
				auto v_data = S + at(5, 0, s_t);
				auto v_data_t = s_t;

				// 因为前面压缩了一行，所以这里复原 V1 到新的内存位置 //
				if (h == 1) {
					v_data[at(0, 0, v_data_t)] = 1.0;
				}
				else if (h == 2) {
					s_mc(1, 2, V1, v1_t, v_data + at(0, 0, v_data_t), v_data_t);
					v_data[at(1, 0, v_data_t)] = -v_data[at(0, 1, v_data_t)];
					v_data[at(1, 1, v_data_t)] = v_data[at(0, 0, v_data_t)];
				}
				else {
					s_mc(h / 2, h, V1, v1_t, v_data + at(0, 0, v_data_t), v_data_t);
					s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, v_data + at(h / 2 + 1, 0, v_data_t), v_data_t);
					s_fill(1, pre_out.mn1, -1.0, v_data + at(h / 2, 0, v_data_t), v_data_t);
					s_fill(1, h - pre_out.mn1, 0.0, v_data + at(h / 2, pre_out.mn1, v_data_t), v_data_t);
					if (pre_out.mn1 == 0)v_data[at(h / 2, 0, v_data_t)] = 1.0;

					for (Size i = -1; ++i < h;) {
						s_nm(h, 1, 1.0 / s_norm(h, v_data + at(0, i, v_data_t), v_data_t), v_data + at(0, i, v_data_t), v_data_t);
					}
				}

				// 因为前面压缩了一行，所以这里复原 V2 到新的内存位置 //
				if (n - h - 1 == 1) {
					v_data[at(0, h, v_data_t)] = 1.0;
				}
				else if (n - h - 1 == 2) {
					s_mc(1, 2, V2, v2_t, v_data + at(0, h, v_data_t), v_data_t);
					v_data[at(1, h + 0, v_data_t)] = -v_data[at(0, h + 1, v_data_t)];
					v_data[at(1, h + 1, v_data_t)] = v_data[at(0, h + 0, v_data_t)];
				}
				else {
					auto k = n - h - 1;
					s_mc(k / 2, k, V2, v2_t, v_data + at(0, h, v_data_t), v_data_t);
					s_mc(k - k / 2 - 1, k, V2 + at(k / 2, 0, v2_t), v2_t, v_data + at(k / 2 + 1, h, v_data_t), v_data_t);
					s_fill(1, pre_out.mn2, -1.0, v_data + at(k / 2, h, v_data_t), v_data_t);
					s_fill(1, k - pre_out.mn2, 0.0, v_data + at(k / 2, h + pre_out.mn2, v_data_t), v_data_t);
					if (pre_out.mn2 == 0)v_data[at(k / 2, h + pre_out.mn2, v_data_t)] = 1.0;

					for (Size i = -1; ++i < k;) {
						s_nm(k, 1, 1.0 / s_norm(k, v_data + at(0, i + h, v_data_t), v_data_t), v_data + at(0, i + h, v_data_t), v_data_t);
					}
				}

				// 使用复原后的 V1 & V2
				V1 = v_data;
				V2 = v_data + at(0, h, v_data_t);
				auto v1_t2 = v_data_t;
				auto v2_t2 = v_data_t;

				//step 8：计算V
				// 若 n < 5，此时将p暂存至局部变量中
				double p_tem[5];
				if (n < 5)	s_mc(1, 5, p, p_t, p_tem, 5);

				// 此时vi的位置也可能会变化
				auto vi = S + at(4, 0, s_t);
				auto vi_t = T(s_t);

				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

						for (Size j = 1; j < n; ++j) {
							vi[at(j, 0, vi_t)] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
						}
					}
					else {
						s_fill(n, 1, 0.0, vi, vi_t);
						vi[at(i, 0, vi_t)] = 1.0;
					}

					// apply deflation
					for (Size j = mn; j < n; ++j) {
						if (mu[at(0, j, mu_t)] == -2) {
						}
						else if (mu[at(0, j, mu_t)] == -1) {
							// type 3
						}
						else {
							// type 4
							auto via = std::cos(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] - std::sin(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
							auto vib = std::sin(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] + std::cos(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
							vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] = via;
							vi[at(j, 0, vi_t)] = vib;
						}
					}

					// apply permutation
					if (n < 5)
						s_permutate_inv(n, 1, RowIterator<Size>(p_tem, 5), vi, vi_t);
					else
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), vi, vi_t);

					// 左乘V1 V2
					s_mm(h, 1, h, V1, v1_t2, vi + at(1, 0, vi_t), vi_t, V + at(0, i, v_t), v_t);
					s_mm(n - h - 1, 1, n - h - 1, V2, v2_t2, vi + at(h + 1, 0, vi_t), vi_t, V + at(h, i, v_t), v_t);
				}

				//step 8：计算S
				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];
						S[at(0, i, s_t)] = base + mu[at(0, i, mu_t)];
					}
					else {
						S[at(0, i, s_t)] = d[at(0, i, d_t)];
					}
				}

#ifdef ARIS_DEBUG_DYNAMIC_SVD
				// make U //
				std::vector<double> _D_U((n + 1)* (n + 1));
				s_mc(n + 1, n, U, u_t, _D_U.data(), n + 1);
				s_mc(3, 1, pre_out.q01.data(), 1, _D_U.data() + n, n + 1);
				s_mc(n + 1 - 3, 1, q, u_t, _D_U.data() + at(3, n, n + 1), n + 1);

				// make V //
				std::vector<double> _D_V(n* n, 0.0);
				s_mc(n / 2, n, V, v_t, _D_V.data() + at(0, 0, n), n);
				s_mc(n - n / 2 - 1, n, V + at(n / 2, 0, v_t), v_t, _D_V.data() + at(n / 2 + 1, 0, n), n);
				s_fill(1, mn, -1.0, _D_V.data() + at(n / 2, 0, n), n);
				s_fill(1, n - mn, 0.0, _D_V.data() + at(n / 2, mn, n), n);
				if (mn == 0)_D_V.data()[at(n / 2, 0, n)] = 1.0;

				for (Size i = -1; ++i < n;) {
					s_nm(n, 1, 1.0 / s_norm(n, _D_V.data() + at(0, i, n), n), _D_V.data() + at(0, i, n), n);
				}

				// make S //
				std::vector<double> _D_S((n + 1) * n);
				s_fill(n + 1, n, 0.0, _D_S.data(), n);
				for (Size i(-1); ++i < n;) {
					_D_S.data()[at(i, i, n)] = S[at(0, i, s_t)];
				}

				std::vector<double> _D_Result((n + 1)* n), _D_Tem((n + 1)* n);
				s_mm(n + 1, n, n + 1, _D_U.data(), _D_S.data(), _D_Tem.data());
				s_mm(n + 1, n, n, _D_Tem.data(), n, _D_V.data(), T(n), _D_Result.data(), n);

				std::vector<double> _D_result_compare((n + 1)*n, 0.0);
				for (Size i(-1); ++i < n;) {
					_D_result_compare[at(i, i, n)] = _D_list.back().S[at(0, i, n)];
					_D_result_compare[at(i + 1, i, n)] = _D_list.back().S[at(1, i, n)];
				}

				if (!s_is_equal(n + 1, n, _D_Result.data(), _D_result_compare.data(), 1e-10)) {
					std::cout << "debug error:" << std::endl;
					std::cout << "debug level:" << _D_list.back().level << "  num:" << _D_list.back().num << std::endl;

					std::cout << "dvc U:" << std::endl;
					dsp(n + 1, n + 1, _D_U.data());
					std::cout << "dvc S:" << std::endl;
					dsp(n + 1, n, _D_S.data());
					std::cout << "dvc V:" << std::endl;
					dsp(n, n, _D_V.data());

					std::cout << "dvc result:" << std::endl;
					dsp(n + 1, n, _D_Result.data());

					std::cout << "dvc result should be:" << std::endl;
					dsp(n + 1, n, _D_result_compare.data());

					std::exit(0);
				}

				_D_list.pop_back();
				if (!_D_list.empty())_D_list.back().num = 1;
#endif
			};

			if (n == 1) {
				auto& a1 = S[at(0, 0, s_t)];
				auto& a2 = S[at(1, 0, s_t)];

				auto theta = std::atan2(a2, a1);
				S[at(0, 0, s_t)] = std::sqrt(a1 * a1 + a2 * a2);

				U[at(0, 0, u_t)] = std::cos(theta);
				U[at(1, 0, u_t)] = std::sin(theta);

				q[at(0, 0, q_t)] = -std::sin(theta);// U_0_1
				q[at(1, 0, q_t)] = std::cos(theta);// U_1_1

#ifdef ARIS_DEBUG_DYNAMIC_SVD
				// make U //
				std::vector<double> _D_U((n + 1)* (n + 1));
				_D_U[at(0, 0, n + 1)] = U[at(0, 0, u_t)];
				_D_U[at(0, 1, n + 1)] = pre_out.q01[0];
				_D_U[at(1, 0, n + 1)] = U[at(1, 0, u_t)];
				_D_U[at(1, 1, n + 1)] = pre_out.q01[1];

				// make V //
				std::vector<double> _D_V(n* n);
				_D_V[at(0, 0, n)] = 1.0;

				// make S //
				std::vector<double> _D_S((n + 1) * n);
				_D_S[at(0, 0, n)] = S[at(0, 0, s_t)];
				_D_S[at(1, 0, n)] = 0.0;

				// verify result //
				std::vector<double> _D_Result((n + 1) * n), _D_Tem((n + 1) * n);
				s_mm(n + 1, n, n + 1, _D_U.data(), _D_S.data(), _D_Tem.data());
				s_mm(n + 1, n, n, _D_Tem.data(), n, _D_V.data(), T(n), _D_Result.data(), n);

				std::vector<double> _D_result_compare((n + 1) * n, 0.0);
				for (Size i(-1); ++i < n;) {
					_D_result_compare[at(i, i, n)] = _D_list.back().S[at(0, i, n)];
					_D_result_compare[at(i + 1, i, n)] = _D_list.back().S[at(1, i, n)];
				}

				if (!s_is_equal(n + 1, n, _D_Result.data(), _D_result_compare.data(), 1e-10)) {
					std::cout << "debug error" << std::endl;

					dsp(n + 1, n, _D_Result.data());
					dsp(n + 1, n, _D_result_compare.data());
				}

				_D_list.pop_back();
				if (!_D_list.empty())_D_list.back().num = 1;
#endif
			}
			else if (n == 2) {
				auto theta1 = std::atan2(S[at(1, 0, s_t)], S[at(0, 0, s_t)]);
				auto c1 = std::cos(theta1);
				auto s1 = std::sin(theta1);

				auto theta2 = std::atan2(S[at(1, 1, s_t)], c1 * S[at(0, 1, s_t)]);
				auto c2 = std::cos(theta2);
				auto s2 = std::sin(theta2);

				auto a = c1 * S[at(0, 0, s_t)] + s1 * S[at(1, 0, s_t)];
				auto b = s1 * S[at(0, 1, s_t)];
				auto d = c2 * c1 * S[at(0, 1, s_t)] + s2 * S[at(1, 1, s_t)];

				auto theta3 = 0.5 * std::atan2(2 * b * d, a * a + b * b - d * d);
				auto c3 = std::cos(theta3);
				auto s3 = std::sin(theta3);

				// S //
				auto S1 = a * a + b * b + d * d;
				auto S2 = std::sqrt((a * a + b * b - d * d) * (a * a + b * b - d * d) + 4 * b * b * d * d);

				S[at(0, 0, s_t)] = std::sqrt((S1 + S2) / 2);
				S[at(0, 1, s_t)] = std::sqrt((S1 - S2) / 2);

				auto phi = 0.5 * std::atan2(2 * a * b, a * a - b * b - d * d);
				auto c_p = std::cos(phi);
				auto s_p = std::sin(phi);
				auto s11 = (a * c3) * c_p + (b * c3 + d * s3) * s_p;
				auto s22 = (a * s3) * s_p + (-b * s3 + d * c3) * c_p;

				// U //
				U[at(0, 0, u_t)] = s_sgn2(s11) * (c1 * c3 - c2 * s1 * s3);
				U[at(0, 1, u_t)] = s_sgn2(s22) * (-c1 * s3 - c2 * c3 * s1);
				U[at(1, 0, u_t)] = s_sgn2(s11) * (c3 * s1 + c1 * c2 * s3);
				U[at(1, 1, u_t)] = s_sgn2(s22) * (c1 * c2 * c3 - s1 * s3);
				U[at(2, 0, u_t)] = s_sgn2(s11) * s2 * s3;
				U[at(2, 1, u_t)] = s_sgn2(s22) * c3 * s2;

				q[at(0, 0, q_t)] = (s1 * s2);
				q[at(1, 0, q_t)] = (-c1 * s2);
				q[at(2, 0, q_t)] = c2;

				// V //
				V[at(0, 0, v_t)] = c_p;
				V[at(0, 1, v_t)] = -s_p;

#ifdef ARIS_DEBUG_DYNAMIC_SVD
				// make U //
				std::vector<double> _D_U((n + 1)* (n + 1));
				_D_U[at(0, 0, n + 1)] = U[at(0, 0, u_t)];
				_D_U[at(0, 1, n + 1)] = U[at(0, 1, u_t)];
				_D_U[at(0, 2, n + 1)] = pre_out.q01[0];
				_D_U[at(1, 0, n + 1)] = U[at(1, 0, u_t)];
				_D_U[at(1, 1, n + 1)] = U[at(1, 1, u_t)];
				_D_U[at(1, 2, n + 1)] = pre_out.q01[1];
				_D_U[at(2, 0, n + 1)] = U[at(2, 0, u_t)];
				_D_U[at(2, 1, n + 1)] = U[at(2, 1, u_t)];
				_D_U[at(2, 2, n + 1)] = pre_out.q01[2];

				// make V //
				std::vector<double> _D_V(n* n);
				_D_V[at(0, 0, n)] = V[at(0, 0, v_t)];
				_D_V[at(0, 1, n)] = V[at(0, 1, v_t)];
				_D_V[at(1, 0, n)] = -_D_V[at(0, 1, n)];
				_D_V[at(1, 1, n)] = _D_V[at(0, 0, n)];

				// make S //
				std::vector<double> _D_S((n + 1)* n, 0.0);
				_D_S[at(0, 0, n)] = S[at(0, 0, s_t)];
				_D_S[at(1, 1, n)] = S[at(0, 1, s_t)];

				// verify result //
				std::vector<double> _D_Result((n + 1)* n), _D_Tem((n + 1)* n);
				s_mm(n + 1, n, n + 1, _D_U.data(), _D_S.data(), _D_Tem.data());
				s_mm(n + 1, n, n, _D_Tem.data(), n, _D_V.data(), T(n), _D_Result.data(), n);

				std::vector<double> _D_result_compare((n + 1)* n, 0.0);
				for (Size i(-1); ++i < n;) {
					_D_result_compare[at(i, i, n)] = _D_list.back().S[at(0, i, n)];
					_D_result_compare[at(i + 1, i, n)] = _D_list.back().S[at(1, i, n)];
				}

				if (!s_is_equal(n + 1, n, _D_Result.data(), _D_result_compare.data(), 1e-10)) {
					std::cout << "debug error:" << std::endl;
					std::cout << "debug level:" << _D_list.back().level << "  num:" << _D_list.back().num << std::endl;

					std::cout << "dvc U:" << std::endl;
					dsp(n + 1, n + 1, _D_U.data());
					std::cout << "dvc S:" << std::endl;
					dsp(n + 1, n, _D_S.data());
					std::cout << "dvc V:" << std::endl;
					dsp(n, n, _D_V.data());

					std::cout << "dvc result:" << std::endl;
					dsp(n + 1, n, _D_Result.data());

					std::cout << "dvc result should be:" << std::endl;
					dsp(n + 1, n, _D_result_compare.data());
				}

				_D_list.pop_back();
				if (!_D_list.empty())_D_list.back().num = 1;
#endif
			}
			else {
				dvc_inside(n, U, u_t, q, q_t, S, s_t, V, v_t, dvc, dvc_pre, pre_out);
			}

			return pre_out;
		};

		//////////////////////////////////////////////////////////// PART 3: 首次调用 /////////////////////////////////////////
		const Size MAX_N = 15;

		if (n < MAX_N) {
			double q[100];
			Size q_t = 1;

			auto pre_out = dvc_pre(n, U, u_t, V, v_t, q, q_t, dvc, dvc_pre);
			s_mc(2, n, U, u_t, S, s_t);

			auto q1 = q;
			auto q2 = q + n/2 + 1;

			auto h = n / 2;
			auto U1 = V + at(0, 0, v_t);
			auto u1_t = v_t;
			auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
			auto v1_t = v_t;
			auto U2 = V + at(h, h, v_t);
			auto u2_t = v_t;
			auto V2 = V + at(h + 1, 0, v_t);
			auto v2_t = v_t;

			auto c0 = pre_out.c0;
			auto s0 = pre_out.s0;
			auto mn = pre_out.mn;
			auto dn = pre_out.dn;

			auto d = S;
			auto d_t = s_t;
			auto z = S + at(1, 0, s_t);
			auto z_t = s_t;

			double mu[MAX_N];
			double p[MAX_N];
			double ui[MAX_N];
			double vi[MAX_N];

			// 初始化mu p
			s_mc(1, n, U + at(3, 0, u_t), u_t, p, MAX_N);
			s_mc(1, n, U + at(2, 0, u_t), u_t, mu, MAX_N);

			// STEP 1: 计算U
			{
				// 计算 U //
				{
					// 计算ui所产生的U（不经过householder变换）
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[i] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 0; j < n; ++j) {
								ui[j] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[i]) / (d[at(0, j, d_t)] + base + mu[i]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, ui);
							ui[i] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[j] == -2) {
							}
							else if (mu[j] == -1) {
								// type 3
								auto ui0 = 0;
								auto uij = j;

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[ui0] - std::sin(z[at(0, j, z_t)]) * ui[uij];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[ui0] + std::cos(z[at(0, j, z_t)]) * ui[uij];
								ui[ui0] = uia;
								ui[uij] = uib;
							}
							else {
								// type 4
								auto k = static_cast<Size>(mu[j]);
								auto uik = k;
								auto uij = j;

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[uik] - std::sin(z[at(0, j, z_t)]) * ui[uij];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[uik] + std::cos(z[at(0, j, z_t)]) * ui[uij];
								ui[uik] = uia;
								ui[uij] = uib;
							}
						}

						// apply permutation
						s_permutate_inv(n, 1, RowIterator<Size>(p, MAX_N), ui, 1);

						// 归一化
						if (i < mn) {
							s_nm(n, 1, 1.0 / s_norm(n, ui, 1), ui, 1);
						}

						// 生成 U，基于U1 //
						s_mc(h + 1, 1, c0 * ui[0], q1, q_t, U + at(0, i, u_t), u_t);
						s_mma(h + 1, 1, h, U1 + at(0, 0, u1_t), u1_t, ui + 1, 1, U + at(0, i, u_t), u_t);

						// 生成 U，基于U2 //
						auto real_m_of_u2 = m == n ? n - h - 1 : n - h;
						s_mc(real_m_of_u2, 1, s0 * ui[0], q2, q_t, U + at(h + 1, i, u_t), u_t);
						s_mma(real_m_of_u2, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + h + 1, 1, U + at(h + 1, i, u_t), u_t);
					}

					// 计算q1&q2所产生的q //
					if (m > n) {
						s_mc(h + 1, 1, -s0, q1, q_t, U + at(0, n, u_t), u_t);
						s_mc(n - h, 1, c0, q2, q_t, U + at(h + 1, n, u_t), u_t);
					}
				}

				// 处理 m > n + 1时额外的方阵
				if (m > n + 1) {
					s_fill(m - n - 1, n + 1, 0.0, U + at(n + 1, 0, u_t), u_t);
					s_fill(n + 1, m - n - 1, 0.0, U + at(0, n + 1, u_t), u_t);
					s_eye(m - n - 1, U + at(n + 1, n + 1, u_t), u_t);
				}

				// 反 householder 变换 //
				s_householder_u_q_dot(m - 1, n, m, S + at(1, 0, s_t), s_t, U + at(1, 0, u_t), u_t, U + at(1, 0, u_t), u_t);
			}

			// STEP 2: 计算V
			{
				// 根据 vi 计算
				{
					// 因为前面压缩了一行，所以这里复原 V1 到新的内存位置 //
					double V1_new[MAX_N*MAX_N / 4], V2_new[MAX_N*MAX_N / 4];
					auto v1_t_new = h;
					auto v2_t_new = n - h - 1;

					if (h == 1) {
						V1_new[at(0, 0, 1)] = 1.0;
					}
					else if (h == 2) {
						s_mc(1, 2, V1, v1_t, V1_new, 2);
						V1_new[at(1, 0, 2)] = -V1_new[at(0, 1, 2)];
						V1_new[at(1, 1, 2)] = V1_new[at(0, 0, 2)];
					}
					else if (h == 3) {
						s_mc(h / 2, h, V1, v1_t, V1_new + at(0, 0, v1_t_new), v1_t_new);
						s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, V1_new + at(h / 2 + 1, 0, v1_t_new), v1_t_new);
						s_fill(1, pre_out.mn1, -1.0, V1_new + at(h / 2, 0, v1_t_new), v1_t_new);
						s_fill(1, h - pre_out.mn1, 0.0, V1_new + at(h / 2, pre_out.mn1, v1_t_new), v1_t_new);
						if (pre_out.mn1 == 0)V1_new[at(h / 2, 0, v1_t_new)] = 1.0;

						for (Size i = -1; ++i < h;) {
							s_nm(h, 1, 1.0 / s_norm(h, V1_new + at(0, i, v1_t_new), v1_t_new), V1_new + at(0, i, v1_t_new), v1_t_new);
						}
					}
					else {
						s_mc(h / 2, h, V1, v1_t, V1_new + at(0, 0, v1_t_new), v1_t_new);
						s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, V1_new + at(h / 2 + 1, 0, v1_t_new), v1_t_new);
						s_fill(1, pre_out.mn1, -1.0, V1_new + at(h / 2, 0, v1_t_new), v1_t_new);
						s_fill(1, h - pre_out.mn1, 0.0, V1_new + at(h / 2, pre_out.mn1, v1_t_new), v1_t_new);
						if (pre_out.mn1 == 0)V1_new[at(h / 2, 0, v1_t_new)] = 1.0;

						for (Size i = -1; ++i < h;) {
							s_nm(h, 1, 1.0 / s_norm(h, V1_new + at(0, i, v1_t_new), v1_t_new), V1_new + at(0, i, v1_t_new), v1_t_new);
						}
					}

					// 因为前面压缩了一行，所以这里复原 V2 到新的内存位置 //
					if (n - h - 1 == 1) {
						V2_new[at(0, 0, 1)] = 1.0;
					}
					else if (n - h - 1 == 2) {
						s_mc(1, 2, V2, v2_t, V2_new + at(0, 0, 2), 2);
						V2_new[at(1, 0, 2)] = -V2_new[at(0, 1, 2)];
						V2_new[at(1, 1, 2)] = V2_new[at(0, 0, 2)];
					}
					else {
						auto k = n - h - 1;
						s_mc(k / 2, k, V2, v2_t, V2_new + at(0, 0, v2_t_new), v2_t_new);
						s_mc(k - k / 2 - 1, k, V2 + at(k / 2, 0, v2_t), v2_t, V2_new + at(k / 2 + 1, 0, v2_t_new), v2_t_new);
						s_fill(1, pre_out.mn2, -1.0, V2_new + at(k / 2, 0, v2_t_new), v2_t_new);
						s_fill(1, k - pre_out.mn2, 0.0, V2_new + at(k / 2, pre_out.mn2, v2_t_new), v2_t_new);
						if (pre_out.mn2 == 0)V2_new[at(k / 2, pre_out.mn2, v2_t_new)] = 1.0;

						for (Size i = -1; ++i < k;) {
							s_nm(k, 1, 1.0 / s_norm(k, V2_new + at(0, i, v2_t_new), v2_t_new), V2_new + at(0, i, v2_t_new), v2_t_new);
						}
					}

					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[i] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 1; j < n; ++j) {
								vi[j] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[i]) / (d[at(0, j, d_t)] + base + mu[i]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, vi, 1);
							vi[i] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[j] == -2) {
							}
							else if (mu[j] == -1) {
								// type 3
							}
							else {
								// type 4
								auto via = std::cos(z[at(0, j, z_t)]) * vi[static_cast<Size>(mu[j])] - std::sin(z[at(0, j, z_t)]) * vi[j];
								auto vib = std::sin(z[at(0, j, z_t)]) * vi[static_cast<Size>(mu[j])] + std::cos(z[at(0, j, z_t)]) * vi[j];
								vi[static_cast<Size>(mu[j])] = via;
								vi[j] = vib;
							}
						}

						// apply permutation
						s_permutate_inv(n, 1, RowIterator<Size>(p, MAX_N), vi, 1);


						if (i < mn) {
							vi[0] = -1.0;
						}

						// 左乘V1
						s_mm(h, 1, h, V1_new, h, vi + 1, 1, V + at(0, i, v_t), v_t);

						// 左乘V2
						s_mm(n - h - 1, 1, n - h - 1, V2_new, n - h - 1, vi + h + 1, 1, V + at(h + 1, i, v_t), v_t);
					}
				}

				// 计算第h行
				for (Size i = 0; i < n; ++i) {
					V[at(h, i, v_t)] = i < mn ? -1.0 : (i == 0 ? 1.0 : 0.0);
					s_nm(n, 1, 1.0 / s_norm(n, V + at(0, i, v_t), v_t), V + at(0, i, v_t), v_t);
				}

				// 反 householder 变换 //
				s_householder_u_q_dot(2, 1, n, householder_vec_n, 1, V + at(n - 2, 0, v_t), v_t, V + at(n - 2, 0, v_t), v_t);
				s_householder_u_q_dot(n, n - 2, n, S + at(2, 0, s_t), T(s_t), V, v_t, V, v_t);
			}

			// STEP 3：计算S
			{
				for (Size i = 0; i < n; ++i) {
					if (i < mn) {
						auto base = mu[i] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];
						S[at(0, i, s_t)] = base + mu[i];
					}
					else {
						S[at(0, i, s_t)] = d[at(0, i, d_t)];
					}
				}

				s_fill(m - 1, n, 0.0, S + at(1, 0, s_t), s_t);
				for (Size i = 0; ++i < n;) {
					S[at(i, i, s_t)] = S[at(0, i, s_t)];
					S[at(0, i, s_t)] = 0.0;
				}
			}
		}
		else {
			double q[100];
			Size q_t = 1;
			
			auto pre_out = dvc_pre(n, U, u_t, V, v_t, q, q_t, dvc, dvc_pre);
			s_mc(2, n, U, u_t, S, s_t);

			auto q1 = q;
			auto q2 = q + n/2 + 1;

			auto h = n / 2;
			auto U1 = V + at(0, 0, v_t);
			auto u1_t = v_t;
			auto V1 = V + at(0, h == n - h ? h : h + 1, v_t);
			auto v1_t = v_t;
			auto U2 = V + at(h, h, v_t);
			auto u2_t = v_t;
			auto V2 = V + at(h + 1, 0, v_t);
			auto v2_t = v_t;

			auto c0 = pre_out.c0;
			auto s0 = pre_out.s0;
			auto mn = pre_out.mn;
			auto dn = pre_out.dn;

			auto d = S;
			auto d_t = s_t;
			auto z = S + at(1, 0, s_t);
			auto z_t = s_t;

			// STEP 1: 计算U
			double r_of_right_corner[12];
			double r_of_tem_memory_for_v1v2[9]; // 为保存v1v2，将householder左下方变成方阵，需保存右上角
			{
				// 根据 U2 计算 //
				{
					auto mu = U + at(2, 0, u_t);
					auto mu_t = u_t;
					auto p = U + at(3, 0, u_t);
					auto p_t = u_t;
					auto ui = U + at(4, 0, u_t);
					auto ui_t = T(u_t);
					
					// 计算U2所产生的U（不经过householder变换）
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 0; j < n; ++j) {
								auto uij = at(j, 0, ui_t);
								ui[uij] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, ui, ui_t);

							auto uii = at(i, 0, ui_t);
							ui[uii] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[at(0, j, mu_t)] == -2) {
							}
							else if (mu[at(0, j, mu_t)] == -1) {
								// type 3
								auto ui0 = at(0, 0, ui_t);
								auto uij = at(j, 0, ui_t);

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[ui0] - std::sin(z[at(0, j, z_t)]) * ui[uij];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[ui0] + std::cos(z[at(0, j, z_t)]) * ui[uij];
								ui[ui0] = uia;
								ui[uij] = uib;
							}
							else {
								// type 4
								auto k = static_cast<Size>(mu[at(0, j, mu_t)]);
								auto uik = at(k, 0, ui_t);
								auto uij = at(j, 0, ui_t);

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[uik] - std::sin(z[at(0, j, z_t)]) * ui[uij];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[uik] + std::cos(z[at(0, j, z_t)]) * ui[uij];
								ui[uik] = uia;
								ui[uij] = uib;
							}
						}

						// apply permutation
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), ui, ui_t);

						if (i < mn) {
							s_nm(n, 1, 1.0 / s_norm(n, ui, ui_t), ui, ui_t);
						}

						// 生成 U，基于U2 //
						auto real_m_of_u2 = m == n ? n - h - 1 : n - h;
						s_mc(real_m_of_u2, 1, s0 * ui[at(0, 0, ui_t)], q2, q_t, U + at(h + 1, i, u_t), u_t);
						s_mma(real_m_of_u2, 1, n - h - 1, U2 + at(0, 0, u2_t), u2_t, ui + at(h + 1, 0, ui_t), ui_t, U + at(h + 1, i, u_t), u_t);
					}

					// 计算q2所产生的q //
					if (m > n) {
						s_mc(n - h, 1, c0, q2, 1, U + at(h + 1, n, u_t), u_t);
					}
				}

				s_fill(n - h, n - h - 1, -1000, U2, v_t);

				// 移动 V1 的第一行，为p腾出位置 //
				// 为 p 和 ui 腾出位置
				s_mc(3, n - h - 1, V2, v_t, V + at(h + 4, n - h - 1, v_t), v_t);

				// 移动 p 到新的位置
				s_mc(2, n, U + at(2, 0, u_t), u_t, V2, v_t);

				// 根据 U1 计算 //
				{
					auto mu = V2;
					auto mu_t = v_t;
					auto p = V2 + at(1, 0, v_t);
					auto p_t = v_t;
					auto ui = V2 + at(2, 0, v_t);
					auto ui_t = T(v_t);

					// 计算U1所产生的U（不经过householder变换）
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 0; j < n; ++j) {
								ui[at(j, 0, ui_t)] = j < mn ? z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, ui, ui_t);
							ui[at(i, 0, ui_t)] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[at(0, j, mu_t)] == -2) {
							}
							else if (mu[at(0, j, mu_t)] == -1) {
								// type 3
								auto uia = std::cos(z[at(0, j, z_t)]) * ui[at(0, 0, ui_t)] - std::sin(z[at(0, j, z_t)]) * ui[at(j, 0, ui_t)];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[at(0, 0, ui_t)] + std::cos(z[at(0, j, z_t)]) * ui[at(j, 0, ui_t)];
								ui[at(0, 0, ui_t)] = uia;
								ui[at(j, 0, ui_t)] = uib;
							}
							else {
								// type 4
								auto idx = static_cast<Size>(mu[at(0, j, mu_t)]);

								auto uia = std::cos(z[at(0, j, z_t)]) * ui[at(idx, 0, ui_t)] - std::sin(z[at(0, j, z_t)]) * ui[at(j, 0, ui_t)];
								auto uib = std::sin(z[at(0, j, z_t)]) * ui[at(idx, 0, ui_t)] + std::cos(z[at(0, j, z_t)]) * ui[at(j, 0, ui_t)];
								ui[at(idx, 0, ui_t)] = uia;
								ui[at(j, 0, ui_t)] = uib;
							}
						}

						// apply permutation
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), ui, ui_t);

						if (i < mn) {
							s_nm(n, 1, 1.0 / s_norm(n, ui, ui_t), ui, ui_t);
						}

						// 生成 U，基于U1 //
						s_mc(h + 1, 1, c0* ui[at(0, 0, ui_t)], q1, q_t, U + at(0, i, u_t), u_t);
						s_mma(h + 1, 1, h, U1 + at(0, 0, u1_t), u1_t, ui + at(1, 0, ui_t), ui_t, U + at(0, i, u_t), u_t);
					}
					// 计算q1所产生的q //
					if (m > n) {
						s_mc(h + 1, 1, -s0, q1, q_t, U + at(0, n, u_t), u_t);
					}
				}

				// 处理 m > n + 1时额外的方阵
				if (m > n + 1) {
					s_fill(m - n - 1, n + 1, 0.0, U + at(n + 1, 0, u_t), u_t);
					s_fill(n + 1, m - n - 1, 0.0, U + at(0, n + 1, u_t), u_t);
					s_eye(m - n - 1, U + at(n + 1, n + 1, u_t), u_t);
				}

				// 对U进行反 householder 变换 //
				s_householder_u_q_dot(m - 1, n, m, S + at(1, 0, s_t), s_t, U + at(1, 0, u_t), u_t, U + at(1, 0, u_t), u_t);

				// 复原压缩的V1，并移动 p 和 mu //
				// 为mu和p开辟内存
				s_mc(3, 4, S + at(n - 3, n - 4, s_t), s_t, r_of_right_corner, 4);
				// 为之后复原V1&V2开辟内存
				s_mc(3, 3, S + at(n - 2 - h, h - 3, s_t), s_t, r_of_tem_memory_for_v1v2, 3);
				// 移动p
				s_mc(1, n, V + at(h + 2, 0, v_t), v_t, S + at(n - 2, 0, s_t), s_t);
				// 移动 mu 到新的位置
				s_mc(1, n, V + at(h + 1, 0, v_t), v_t, S + at(n - 1, 0, s_t), s_t);
				// 复原 V1
				s_mc(3, n - h - 1, V + at(h + 4, n - h - 1, v_t), v_t, V2, v2_t);
			}

			// STEP 2: 计算V
			{
				auto mu = S + at(n - 1, 0, s_t);
				auto mu_t = s_t;

				// 根据 V1 计算
				{
					// 因为前面压缩了一行，所以这里复原 V1 到新的内存位置 //
					double V1_data_when_n_less_8_[9];

					auto V1_new = S + at(n - 2 - h, 0, s_t);
					auto v1_t_new = s_t;

					if (h == 1) {
						V1_data_when_n_less_8_[at(0, 0, 1)] = 1.0;
					}
					else if (h == 2) {
						s_mc(1, 2, V1, v1_t, V1_data_when_n_less_8_, 2);
						V1_data_when_n_less_8_[at(1, 0, 2)] = -V1_data_when_n_less_8_[at(0, 1, 2)];
						V1_data_when_n_less_8_[at(1, 1, 2)] = V1_data_when_n_less_8_[at(0, 0, 2)];
					}
					else if (h == 3) {

						V1_new = V1_data_when_n_less_8_;
						auto v1_t_new = 3;

						s_mc(h / 2, h, V1, v1_t, V1_new + at(0, 0, v1_t_new), v1_t_new);
						s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, V1_new + at(h / 2 + 1, 0, v1_t_new), v1_t_new);
						s_fill(1, pre_out.mn1, -1.0, V1_new + at(h / 2, 0, v1_t_new), v1_t_new);
						s_fill(1, h - pre_out.mn1, 0.0, V1_new + at(h / 2, pre_out.mn1, v1_t_new), v1_t_new);
						if (pre_out.mn1 == 0)V1_new[at(h / 2, 0, v1_t_new)] = 1.0;

						for (Size i = -1; ++i < h;) {
							s_nm(h, 1, 1.0 / s_norm(h, V1_new + at(0, i, v1_t_new), v1_t_new), V1_new + at(0, i, v1_t_new), v1_t_new);
						}
					}
					else {
						s_mc(h / 2, h, V1, v1_t, V1_new + at(0, 0, v1_t_new), v1_t_new);
						s_mc(h - h / 2 - 1, h, V1 + at(h / 2, 0, v1_t), v1_t, V1_new + at(h / 2 + 1, 0, v1_t_new), v1_t_new);
						s_fill(1, pre_out.mn1, -1.0, V1_new + at(h / 2, 0, v1_t_new), v1_t_new);
						s_fill(1, h - pre_out.mn1, 0.0, V1_new + at(h / 2, pre_out.mn1, v1_t_new), v1_t_new);
						if (pre_out.mn1 == 0)V1_new[at(h / 2, 0, v1_t_new)] = 1.0;

						for (Size i = -1; ++i < h;) {
							s_nm(h, 1, 1.0 / s_norm(h, V1_new + at(0, i, v1_t_new), v1_t_new), V1_new + at(0, i, v1_t_new), v1_t_new);
						}
					}

					auto vi = V + at(h, 0, v_t);
					auto vi_t = T(v_t);
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 1; j < n; ++j) {
								vi[at(j, 0, vi_t)] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, vi, vi_t);
							vi[at(i, 0, vi_t)] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[at(0, j, mu_t)] == -2) {
							}
							else if (mu[at(0, j, mu_t)] == -1) {
								// type 3
							}
							else {
								// type 4
								auto via = std::cos(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] - std::sin(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
								auto vib = std::sin(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] + std::cos(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
								vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] = via;
								vi[at(j, 0, vi_t)] = vib;
							}
						}

						// apply permutation
						auto p = S + at(n - 2, 0, s_t);
						auto p_t = s_t;
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), vi, vi_t);

						if (i < mn) {
							vi[at(0, 0, vi_t)] = -1.0;
							//s_nm(n, 1, 1.0 / s_norm(n, vi, vi_t), vi, vi_t);
						}

						// 左乘V1
						V1 = V1_new;
						auto v1_t2 = v1_t_new;

						s_mm(h, 1, h, V1, v1_t2, vi + at(1, 0, vi_t), vi_t, V + at(0, i, v_t), v_t);
					}
				}

				// 根据 V2 计算
				{
					double V1_data_when_n_less_8_[9];

					auto V2_new = S + at(n - 2 - h, 0, s_t);
					auto v2_t_new = s_t;

					// 因为前面压缩了一行，所以这里复原 V2 到新的内存位置 //
					if (n - h - 1 == 1) {
						V1_data_when_n_less_8_[at(0, 0, 1)] = 1.0;
					}
					else if (n - h - 1 == 2) {
						s_mc(1, 2, V2, v2_t, V1_data_when_n_less_8_ + at(0, 0, 2), 2);
						V1_data_when_n_less_8_[at(1, 0, 2)] = -V1_data_when_n_less_8_[at(0, 1, 2)];
						V1_data_when_n_less_8_[at(1, 1, 2)] = V1_data_when_n_less_8_[at(0, 0, 2)];
					}
					else {
						auto k = n - h - 1;
						s_mc(k / 2, k, V2, v2_t, V2_new + at(0, 0, v2_t_new), v2_t_new);
						s_mc(k - k / 2 - 1, k, V2 + at(k / 2, 0, v2_t), v2_t, V2_new + at(k / 2 + 1, 0, v2_t_new), v2_t_new);
						s_fill(1, pre_out.mn2, -1.0, V2_new + at(k / 2, 0, v2_t_new), v2_t_new);
						s_fill(1, k - pre_out.mn2, 0.0, V2_new + at(k / 2, pre_out.mn2, v2_t_new), v2_t_new);
						if (pre_out.mn2 == 0)V2_new[at(k / 2, pre_out.mn2, v2_t_new)] = 1.0;

						for (Size i = -1; ++i < k;) {
							s_nm(k, 1, 1.0 / s_norm(k, V2_new + at(0, i, v2_t_new), v2_t_new), V2_new + at(0, i, v2_t_new), v2_t_new);
						}
					}

					auto vi = V + at(h, 0, v_t);
					auto vi_t = T(v_t);
					for (Size i = 0; i < n; ++i) {
						if (i < mn) {
							auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];

							for (Size j = 1; j < n; ++j) {
								vi[at(j, 0, vi_t)] = j < mn ? d[at(0, j, d_t)] * z[at(0, j, z_t)] / (d[at(0, j, d_t)] - base - mu[at(0, i, mu_t)]) / (d[at(0, j, d_t)] + base + mu[at(0, i, mu_t)]) : 0.0;
							}
						}
						else {
							s_fill(n, 1, 0.0, vi, vi_t);
							vi[at(i, 0, vi_t)] = 1.0;
						}

						// apply deflation
						for (Size j = mn; j < n; ++j) {
							if (mu[at(0, j, mu_t)] == -2) {
							}
							else if (mu[at(0, j, mu_t)] == -1) {
								// type 3
							}
							else {
								// type 4
								auto via = std::cos(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] - std::sin(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
								auto vib = std::sin(z[at(0, j, z_t)]) * vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] + std::cos(z[at(0, j, z_t)]) * vi[at(j, 0, vi_t)];
								vi[at(static_cast<Size>(mu[at(0, j, mu_t)]), 0, vi_t)] = via;
								vi[at(j, 0, vi_t)] = vib;
							}
						}

						// apply permutation
						auto p = S + at(n - 2, 0, s_t);
						auto p_t = s_t;
						s_permutate_inv(n, 1, RowIterator<decltype(p_t)>(p, p_t), vi, vi_t);

						if (i < mn) {
							vi[at(0, 0, vi_t)] = -1.0;
						}

						// 左乘V2
						V2 = V2_new;
						auto v2_t2 = v2_t_new;
						s_mm(n - h - 1, 1, n - h - 1, V2, v2_t2, vi + at(h + 1, 0, vi_t), vi_t, V + at(h + 1, i, v_t), v_t);
					}
				}

				// 计算第h行
				for (Size i = 0; i < n; ++i) {
					V[at(h, i, v_t)] = i < mn ? -1.0 : (i == 0 ? 1.0 : 0.0);
					s_nm(n, 1, 1.0 / s_norm(n, V + at(0, i, v_t), v_t), V + at(0, i, v_t), v_t);
				}

				// 复原householders //
				double r_of_right_corner_2[12];
				s_mc(3, 4, S + at(n - 3, n - 4, s_t), s_t, r_of_right_corner_2, 4);

				s_mc(3, 4, r_of_right_corner, 4, S + at(n - 3, n - 4, s_t), s_t);
				s_mc(3, 3, r_of_tem_memory_for_v1v2, 3, S + at(n - 2 - h, h - 3, s_t), s_t);


				// 反 householder 变换 //
				s_householder_u_q_dot(2, 1, n, householder_vec_n, 1, V + at(n - 2, 0, v_t), v_t, V + at(n - 2, 0, v_t), v_t);
				s_householder_u_q_dot(n, n - 2, n, S + at(2, 0, s_t), T(s_t), V, v_t, V, v_t);

				// 复原 mu //
				s_mc(3, 4, r_of_right_corner_2, 4, S + at(n - 3, n - 4, s_t), s_t);
			}

			// STEP 3：计算S
			auto mu = S + at(n - 1, 0, s_t);
			auto mu_t = s_t;

			for (Size i = 0; i < n; ++i) {
				if (i < mn) {
					auto base = mu[at(0, i, mu_t)] < 0.0 ? (i == mn - 1 ? dn : d[at(0, i + 1, d_t)]) : d[at(0, i, d_t)];
					S[at(0, i, s_t)] = base + mu[at(0, i, mu_t)];
				}
				else {
					S[at(0, i, s_t)] = d[at(0, i, d_t)];
				}
			}

			s_fill(m - 1, n, 0.0, S + at(1, 0, s_t), s_t);
			for (Size i = 0; ++i < n;) {
				S[at(i, i, s_t)] = S[at(0, i, s_t)];
				S[at(0, i, s_t)] = 0.0;
			}
		}

#ifdef ARIS_DEBUG_DYNAMIC_SVD
		// should check result
#endif
	}
	
}

void test_basic_operation()
{
	//test isEqual
	{
		double a[] = { 0.1,0.2,0.3 };
		double b[] = { 0.1,0.2,0.3 };
		double c[] = { 0.1,0.2,0.35 };

		if (!s_is_equal(3, a, b, error))std::cout << "\"s_is_equal\" failed" << std::endl;
		if (s_is_equal(3, a, c, error))std::cout << "\"s_is_equal\" failed" << std::endl;

		double d[]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
		double e[]{ 0.1,0.0,0.2,0.3,0.0,0.4,0.5,0.0,0.6 };
		if (!s_is_equal(3, d, 2, e, 3, error))std::cout << "\"s_is_equal\" failed" << std::endl;
		if (s_is_equal(3, d, 2, e + 1, 3, error))std::cout << "\"s_is_equal\" failed" << std::endl;

		double m1[]{ 0.1,0.2,0.3,
			0.4,0.5,0.6 };
		double m2[]{ 0.1,0.2,0.3,
			0.4,0.5,0.6 };
		double m3[]{ 0.1,0.2,0.31,
			0.4,0.5,0.6 };

		if (!s_is_equal(2, 3, m1, m2, error))std::cout << "\"s_is_equal mtx\" failed" << std::endl;
		if (s_is_equal(2, 3, m1, m3, error))std::cout << "\"s_is_equal mtx\" failed" << std::endl; 
		
		double m4[]{ 0.1,0.2,0.3,0.0,
			0.4,0.5,0.6,0.0 };
		double m5[]{ 0.1,0.2,0.3,0.0,0.0,
			0.4,0.5,0.6,0.0,0.0, };
		double m6[]{ 0.1,0.2,0.3,0.0,0.0,
			0.4,0.51,0.6,0.0,0.0, };

		if (!s_is_equal(2, 3, m4, 4, m5, 5, error))std::cout << "\"s_is_equal mtx\" failed" << std::endl;
		if (s_is_equal(2, 3, m4, 4, m6, 5, error))std::cout << "\"s_is_equal mtx\" failed" << std::endl;
	}

}
void test_specific_matrix(){
	double result[36];
	
	const double eye3[9]{ 1,0,0,0,1,0,0,0,1 };

	s_eye(3, result);
	if (!s_is_equal(9, result, eye3, error))std::cout << __FILE__ << __LINE__ << " failed" << std::endl;
	s_eye(3, result, 4);
	if (!s_is_equal(3, 3, result, 4, eye3, 3, error))std::cout << __FILE__ << __LINE__ << " failed" << std::endl;

	const double rmx[9]{ 1,0,0,0,0.696706709347165, - 0.717356090899523,0,0.717356090899523,0.696706709347165 };
	const double rmy[9]{ 0.696706709347165,0,0.717356090899523,	0,1,0,- 0.717356090899523,0,0.696706709347165 };
	const double rmz[9]{ 0.696706709347165, - 0.717356090899523,0,0.717356090899523,0.696706709347165,0,0,0,1 };

	s_rmx(0.8, result);
	if (!s_is_equal(9, result, rmx, error))std::cout << __FILE__ << __LINE__ << " failed" << std::endl;
	s_rmx(0.8, result, 4);
	if (!s_is_equal(3, 3, result, 4, rmx, 3, error))std::cout << __FILE__ << __LINE__ << " failed" << std::endl;

	s_rmy(0.8, result);
	if (!s_is_equal(9, result, rmy, error))std::cout << __FILE__ << __LINE__ << " failed" << std::endl;
	s_rmy(0.8, result, 4);
	if (!s_is_equal(3, 3, result, 4, rmy, 3, error))std::cout << __FILE__ << __LINE__ << " failed" << std::endl;

	s_rmz(0.8, result);
	if (!s_is_equal(9, result, rmz, error))std::cout << __FILE__ << __LINE__ << " failed" << std::endl;
	s_rmz(0.8, result, 4);
	if (!s_is_equal(3, 3, result, 4, rmz, 3, error))std::cout << __FILE__ << __LINE__ << " failed" << std::endl;
}
void test_multiply(){
	const double alpha{ 0.255095115459269 };
	const double v[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double v_ld[]{ 0.498364051982143,0,0,0.959743958516081,0,0,0.340385726666133,0,0,0.585267750979777,0,0,0.223811939491137,0,0,0.751267059305653,0,0 };
	const double r1[]{ 0.127130235381134,0.244825995908996,0.0868307362445845,0.149298944510773,0.0570933325456545,0.191644557234321 };
	const double r1_ld[]{ 0.127130235381134,0,0,0.244825995908996,0,0,0.0868307362445845,0,0,0.149298944510773,0,0,0.0570933325456545,0,0,0.191644557234321,0,0 };
	const double r2[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double r2_ld[]{ 0.498364051982143,0,0.959743958516081,0,0.340385726666133,0,0.585267750979777,0,0.223811939491137,0,0.751267059305653,0 };
	const double r3[]{ 0.127130235381134,0.244825995908996,0.0868307362445845,0.149298944510773,0.0570933325456545,0.191644557234321 };
	const double r3_ld[]{ 0.127130235381134,0,0.244825995908996,0,0.0868307362445845,0,0.149298944510773,0,0.0570933325456545,0,0.191644557234321,0 };
	const double r4[]{ 1.00432110364729,1.65882068117277,1.23128897920193,1.54455917618522,0.77102746945494,0.889891502134332 };
	double r4_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0.959291425205444,0.547215529963803,0.138624442828679 };
	const double r4_ld[]{ 1.00432110364729,0,1.65882068117277,0,1.23128897920193,0,1.54455917618522,0,0.77102746945494,0,0.889891502134332,0 };
	double r4_ld_before[]{ 0.505957051665142,0,0.699076722656686,0,0.890903252535799,0,0.959291425205444,0,0.547215529963803,0,0.138624442828679,0 };
	const double r5[]{ 0.633087287046276,0.943902718565682,0.977733988780383,1.10859036971622,0.604308862509458,0.330269000063 };
	double r5_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0.959291425205444,0.547215529963803,0.138624442828679 };
	const double r5_ld[]{ 0.633087287046276,0,0.943902718565682,0,0.977733988780383,0,1.10859036971622,0,0.604308862509458,0,0.330269000063,0 };
	double r5_ld_before[]{ 0.505957051665142,0,0.699076722656686,0,0.890903252535799,0,0.959291425205444,0,0.547215529963803,0,0.138624442828679,0 };
	const double r6[]{ -0.498364051982143,-0.959743958516081,-0.340385726666133,-0.585267750979777,-0.223811939491137,-0.751267059305653 };
	const double r6_ld[]{ -0.498364051982143,0,0,-0.959743958516081,0,0,-0.340385726666133,0,0,-0.585267750979777,0,0,-0.223811939491137,0,0,-0.751267059305653,0,0 };
	const double r7[]{ 0.134723235064133, - 0.015841239950399,0.63734826211425,0.523322618736443,0.380496923018321, - 0.420998059242653 };
	double r7_before[]{ 0.633087287046276,0.943902718565682,0.977733988780383,1.10859036971622,0.604308862509458,0.330269000063 };
	const double r7_ld[]{ 0.134723235064133,0, -0.015841239950399,0,0.63734826211425,0,0.523322618736443,0,0.380496923018321,0, -0.420998059242653,0 };
	double r7_ld_before[]{ 0.633087287046276,0,0.943902718565682,0,0.977733988780383,0,1.10859036971622,0,0.604308862509458,0,0.330269000063,0 };


	const double v1[]{ 0.1,0.23,0.45,0.67,0.89 };
	const double v2[]{ -0.1,0.89,-0.45,0.67,0.12 };
	const double v1_ld[]{ 0.1,0,0.23,0,0.45,0,0.67,0,0.89,0 };
	const double v2_ld[]{ -0.1,0,0,0.89,0,0,-0.45,0,0,0.67,0,0,0.12,0,0 };
	double vv_result{ 0.5479 };


	double result[36];

	std::copy(v, v + 6, result);
	s_nv(6, alpha, result);
	if (!s_is_equal(6, result, r1, error))std::cout << "\"s_nv\" failed" << std::endl;

	std::copy(v_ld, v_ld + 18, result);
	s_nv(6, alpha, result, 3);
	if (!s_is_equal(18, result, r1_ld, error))std::cout << "\"s_nv with ld\" failed" << std::endl;

	std::copy(v, v + 6, result);
	s_iv(6, result);
	if (!s_is_equal(6, result, r6, error))std::cout << "\"s_iv\" failed" << std::endl;

	std::copy(v_ld, v_ld + 18, result);
	s_iv(6, result, 3);
	if (!s_is_equal(18, result, r6_ld, error))std::cout << "\"s_iv with ld\" failed" << std::endl;

	std::fill(result, result + 36, 1.0);
	s_vc(6, v, result);
	if (!s_is_equal(6, result, r2, error))std::cout << "\"s_vc\" failed" << std::endl;

	std::fill(result, result + 36, 0);
	s_vc(6, v_ld, 3, result, 2);
	if (!s_is_equal(12, result, r2_ld, error))std::cout << "\"s_vc with ld\" failed" << std::endl;

	std::fill(result, result + 36, 1.0);
	s_vi(6, v, result);
	if (!s_is_equal(6, result, r6, error))std::cout << "\"s_vi\" failed" << std::endl;

	std::fill(result, result + 36, 0);
	s_vi(6, v_ld, 3, result, 3);
	if (!s_is_equal(18, result, r6_ld, error))std::cout << "\"s_vi with ld\" failed" << std::endl;

	std::fill(result, result + 36, 1.0);
	s_vc(6, alpha, v, result);
	if (!s_is_equal(6, result, r3, error))std::cout << "\"s_vc\" failed" << std::endl;

	std::fill(result, result + 36, 0);
	s_vc(6, alpha, v_ld, 3, result, 2);
	if (!s_is_equal(12, result, r3_ld, error))std::cout << "\"s_vc with ld\" failed" << std::endl;

	s_va(6, v, r4_before);
	if (!s_is_equal(6, r4_before, r4, error))std::cout << "\"s_va\" failed" << std::endl;

	s_va(6, v_ld, 3, r4_ld_before, 2);
	if (!s_is_equal(12, r4_ld_before, r4_ld, error))std::cout << "\"s_va with ld\" failed" << std::endl;

	s_va(6, alpha, v, r5_before);
	if (!s_is_equal(6, r5_before, r5, error))std::cout << "\"s_va\" failed" << std::endl;

	s_va(6, alpha, v_ld, 3, r5_ld_before, 2);
	if (!s_is_equal(12, r5_ld_before, r5_ld, error))std::cout << "\"s_va with ld\" failed" << std::endl;

	s_vs(6, v, r7_before);
	if (!s_is_equal(6, r7_before, r7, error))std::cout << "\"s_vs\" failed" << std::endl;

	s_vs(6, v_ld, 3, r7_ld_before, 2);
	if (!s_is_equal(12, r7_ld_before, r7_ld, error))std::cout << "\"s_vs with ld\" failed" << std::endl;

	result[0] = s_vv(5, v1, v2);
	if (!s_is_equal(1, result, &vv_result, error))std::cout << "\"s_vv\" failed" << std::endl;

	result[0] = s_vv(5, v1_ld, 2, v2_ld, 3);
	if (!s_is_equal(1, result, &vv_result, error))std::cout << "\"s_vv with ld\" failed" << std::endl;


	const double m1[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double o1[]{ 0.127130235381134,0.244825995908996,0.0868307362445845,0.149298944510773,0.0570933325456545,0.191644557234321 };
	const double m2[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double o2[]{ 0.127130235381134,0.244825995908996,0.0868307362445845,0,0,0,0.149298944510773,0.0570933325456545,0.191644557234321,0,0,0 };

	const double m2_ld[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0 };
	const double m1T[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double m2T_ld[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };

	const double m3[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double o3[]{ 1.00432110364729,1.65882068117277,1.23128897920193,1.54455917618522,0.77102746945494,0.889891502134332 };
	double o3_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0.959291425205444,0.547215529963803,0.138624442828679 };
	const double m4[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0 };
	const double o4[]{ 1.00432110364729,1.65882068117277,1.23128897920193,0,1.54455917618522,0.77102746945494,0.889891502134332,0 };
	double o4_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0,0.959291425205444,0.547215529963803,0.138624442828679,0 };
	const double m5[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double o5[]{ 0.633087287046276,0.943902718565682,0.977733988780383,1.10859036971622,0.604308862509458,0.330269000063 };
	double o5_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0.959291425205444,0.547215529963803,0.138624442828679 };
	const double m6[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0 };
	const double o6[]{ 0.633087287046276,0.943902718565682,0.977733988780383,0,1.10859036971622,0.604308862509458,0.330269000063,0 };
	double o6_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0,0.959291425205444,0.547215529963803,0.138624442828679,0 };

	const double m3T[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	double o3T_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0.959291425205444,0.547215529963803,0.138624442828679 };
	const double m4T[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	double o4T_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0,0.959291425205444,0.547215529963803,0.138624442828679,0 };
	const double m5T[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	double o5T_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0.959291425205444,0.547215529963803,0.138624442828679 };
	const double m6T[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	double o6T_before[]{ 0.505957051665142,0.699076722656686,0.890903252535799,0,0.959291425205444,0.547215529963803,0.138624442828679,0 };

	const double m7a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double m7b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double o7[]{ 1.31858183661872,1.09529802045975,0.989581394872564,0.962509892352645,0.774959561865027,0.749377875701963,1.0047379842151,1.32916821737827 };
	const double m8a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double m8b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double o8[]{ 1.31858183661872,1.09529802045975,0.989581394872564,0.962509892352645,0,0.774959561865027,0.749377875701963,1.0047379842151,1.32916821737827,0 };
	const double m9a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double m9b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double o9[]{ 0.336363785854748,0.27940517499149,0.252437380181361,0.245531572120387,0.197688398910224,0.191162635724814,0.256303752089665,0.339064319876901 };
	const double m10a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double m10b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double o10[]{ 0.336363785854748,0.27940517499149,0.252437380181361,0.245531572120387,0,0.197688398910224,0.191162635724814,0.256303752089665,0.339064319876901,0 };

	const double m11a[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double m11b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double o11[]{ 1.31858183661872,1.09529802045975,0.989581394872564,0.962509892352645,0.774959561865027,0.749377875701963,1.0047379842151,1.32916821737827 };
	const double m12a[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	const double m12b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double o12[]{ 1.31858183661872,1.09529802045975,0.989581394872564,0.962509892352645,0,0.774959561865027,0.749377875701963,1.0047379842151,1.32916821737827,0 };
	const double m13a[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double m13b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double o13[]{ 0.336363785854748,0.27940517499149,0.252437380181361,0.245531572120387,0.197688398910224,0.191162635724814,0.256303752089665,0.339064319876901 };
	const double m14a[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	const double m14b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double o14[]{ 0.336363785854748,0.27940517499149,0.252437380181361,0.245531572120387,0,0.197688398910224,0.191162635724814,0.256303752089665,0.339064319876901,0 };

	const double m15a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double m15b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0.913375856139019,0.63235924622541,0.0975404049994095,0.278498218867048,0.546881519204984,0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616 };
	const double o15[]{ 1.31858183661872,1.09529802045975,0.989581394872564,0.962509892352645,0.774959561865027,0.749377875701963,1.0047379842151,1.32916821737827 };
	const double m16a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double m16b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0,0,0.913375856139019,0.63235924622541,0.0975404049994095,0,0,0.278498218867048,0.546881519204984,0.957506835434298,0,0,0.964888535199277,0.157613081677548,0.970592781760616,0,0 };
	const double o16[]{ 1.31858183661872,1.09529802045975,0.989581394872564,0.962509892352645,0,0.774959561865027,0.749377875701963,1.0047379842151,1.32916821737827,0 };
	const double m17a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double m17b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0.913375856139019,0.63235924622541,0.0975404049994095,0.278498218867048,0.546881519204984,0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616 };
	const double o17[]{ 0.336363785854748,0.27940517499149,0.252437380181361,0.245531572120387,0.197688398910224,0.191162635724814,0.256303752089665,0.339064319876901 };
	const double m18a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double m18b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0,0,0.913375856139019,0.63235924622541,0.0975404049994095,0,0,0.278498218867048,0.546881519204984,0.957506835434298,0,0,0.964888535199277,0.157613081677548,0.970592781760616,0,0 };
	const double o18[]{ 0.336363785854748,0.27940517499149,0.252437380181361,0.245531572120387,0,0.197688398910224,0.191162635724814,0.256303752089665,0.339064319876901,0 };

	const double m19a[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double m19b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0.913375856139019,0.63235924622541,0.0975404049994095,0.278498218867048,0.546881519204984,0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616 };
	const double o19[]{ 1.31858183661872,1.09529802045975,0.989581394872564,0.962509892352645,0.774959561865027,0.749377875701963,1.0047379842151,1.32916821737827 };
	const double m20a[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	const double m20b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0,0,0.913375856139019,0.63235924622541,0.0975404049994095,0,0,0.278498218867048,0.546881519204984,0.957506835434298,0,0,0.964888535199277,0.157613081677548,0.970592781760616,0,0 };
	const double o20[]{ 1.31858183661872,1.09529802045975,0.989581394872564,0.962509892352645,0,0.774959561865027,0.749377875701963,1.0047379842151,1.32916821737827,0 };
	const double m21a[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double m21b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0.913375856139019,0.63235924622541,0.0975404049994095,0.278498218867048,0.546881519204984,0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616 };
	const double o21[]{ 0.336363785854748,0.27940517499149,0.252437380181361,0.245531572120387,0.197688398910224,0.191162635724814,0.256303752089665,0.339064319876901 };
	const double m22a[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	const double m22b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0,0,0.913375856139019,0.63235924622541,0.0975404049994095,0,0,0.278498218867048,0.546881519204984,0.957506835434298,0,0,0.964888535199277,0.157613081677548,0.970592781760616,0,0 };
	const double o22[]{ 0.336363785854748,0.27940517499149,0.252437380181361,0.245531572120387,0,0.197688398910224,0.191162635724814,0.256303752089665,0.339064319876901,0 };

	const double m23[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double o23[]{ -0.498364051982143,-0.959743958516081,-0.340385726666133,-0.585267750979777,-0.223811939491137,-0.751267059305653 };
	const double m24[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double o24[]{ -0.498364051982143,-0.959743958516081,-0.340385726666133,0,0,0,-0.585267750979777,-0.223811939491137,-0.751267059305653,0,0,0 };

	const double m25[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double o25[]{ 0.134723235064133, -0.015841239950399,0.63734826211425,0.523322618736443,0.380496923018321, -0.420998059242653 };
	double o25_before[]{ 0.633087287046276,0.943902718565682,0.977733988780383,1.10859036971622,0.604308862509458,0.330269000063 };
	const double m26[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0 };
	const double o26[]{ 0.134723235064133, -0.015841239950399,0.63734826211425,0,0.523322618736443,0.380496923018321, -0.420998059242653,0 };
	double o26_before[]{ 0.633087287046276,0.943902718565682,0.977733988780383,0,1.10859036971622,0.604308862509458,0.330269000063,0 };

	const double m27a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double m27b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double o27[]{ -1.31858183661872,-1.09529802045975,-0.989581394872564,-0.962509892352645,-0.774959561865027,-0.749377875701963,-1.0047379842151,-1.32916821737827 };
	const double m28a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double m28b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double o28[]{ -1.31858183661872,-1.09529802045975,-0.989581394872564,-0.962509892352645,0,-0.774959561865027,-0.749377875701963,-1.0047379842151,-1.32916821737827,0 };


	std::copy(m1, m1 + 6, result);
	s_nm(2, 3, alpha, result);
	if (!s_is_equal(6, result, o1, error))std::cout << "\"s_nm\" failed" << std::endl;

	std::copy(m2, m2 + 12, result);
	s_nm(2, 3, alpha, result, 6);
	if (!s_is_equal(12, result, o2, error))std::cout << "\"s_nm with ld\" failed" << std::endl;

	std::copy(m23, m23 + 6, result);
	s_im(2, 3, result);
	if (!s_is_equal(6, result, o23, error))std::cout << "\"s_im\" failed" << std::endl;

	std::copy(m24, m24 + 12, result);
	s_im(2, 3, result, 6);
	if (!s_is_equal(12, result, o24, error))std::cout << "\"s_im with ld\" failed" << std::endl;

	s_mc(2, 3, m1, result);
	if (!s_is_equal(6, result, m1, error))std::cout << "\"s_mc\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mc(2, 3, m2_ld, 5, result, 6);
	if (!s_is_equal(12, result, m2, error))std::cout << "\"s_mc with ld\" failed" << std::endl;

	s_mc(2, 3, alpha, m1, result);
	if (!s_is_equal(6, result, o1, error))std::cout << "\"s_mc\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mc(2, 3, alpha, m2_ld, 5, result, 6);
	if (!s_is_equal(12, result, o2, error))std::cout << "\"s_mc with ld\" failed" << std::endl;

	s_mc(2, 3, m1T, ColMajor{2}, result, 3);
	if (!s_is_equal(6, result, m1, error))std::cout << "\"s_mcT\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mc(2, 3, m2T_ld, ColMajor{ 5 }, result, 6);
	if (!s_is_equal(12, result, m2, error))std::cout << "\"s_mcT with ld\" failed" << std::endl;

	s_mc(2, 3, alpha, m1T, ColMajor{ 2 }, result, 3);
	if (!s_is_equal(6, result, o1, error))std::cout << "\"s_mcT\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mc(2, 3, alpha, m2T_ld, ColMajor{ 5 }, result, 6);
	if (!s_is_equal(12, result, o2, error))std::cout << "\"s_mcT with ld\" failed" << std::endl;

	s_mi(2, 3, m23, result);
	if (!s_is_equal(6, result, o23, error))std::cout << "\"s_mi\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mi(2, 3, m23, 3, result, 6);
	if (!s_is_equal(12, result, o24, error))std::cout << "\"s_mi with ld\" failed" << std::endl;

	s_ma(2, 3, m3, o3_before);
	if (!s_is_equal(6, o3, o3_before, error))std::cout << "\"s_ma\" failed" << std::endl;

	s_ma(2, 3, m26, 5, o4_before, 4);
	if (!s_is_equal(8, o4, o4_before, error))std::cout << "\"s_ma with ld\" failed" << std::endl;

	s_ma(2, 3, alpha, m5, o5_before);
	if (!s_is_equal(6, o5, o5_before, error))std::cout << "\"s_ma\" failed" << std::endl;

	s_ma(2, 3, alpha, m6, 5, o6_before, 4);
	if (!s_is_equal(8, o6, o6_before, error))std::cout << "\"s_ma with ld\" failed" << std::endl;

	s_ma(2, 3, m3T, ColMajor{2}, o3T_before, 3);
	if (!s_is_equal(6, o3, o3T_before, error))std::cout << "\"s_maT\" failed" << std::endl;

	s_ma(2, 3, m4T, ColMajor{ 5 }, o4T_before, 4);
	if (!s_is_equal(8, o4, o4T_before, error))std::cout << "\"s_maT with ld\" failed" << std::endl;

	s_ma(2, 3, alpha, m5T, ColMajor{2}, o5T_before, 3);
	if (!s_is_equal(6, o5, o5T_before, error))std::cout << "\"s_maT\" failed" << std::endl;

	s_ma(2, 3, alpha, m6T, ColMajor{ 5 }, o6T_before, 4);
	if (!s_is_equal(8, o6, o6T_before, error))std::cout << "\"s_maT with ld\" failed" << std::endl;

	s_ms(2, 3, m3, o25_before);
	if (!s_is_equal(6, o25, o25_before, error))std::cout << "\"s_ms\" failed" << std::endl;

	s_ms(2, 3, m4, 5, o26_before, 4);
	if (!s_is_equal(8, o26, o26_before, error))std::cout << "\"s_ms with ld\" failed" << std::endl;

	s_mm(2, 4, 3, m7a, m7b, result);
	if (!s_is_equal(8, o7, result, error))std::cout << "\"s_mm\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mm(2, 4, 3, m8a, 6, m8b, 6, result, 5);
	if (!s_is_equal(10, o8, result, error))std::cout << "\"s_mm with ld\" failed" << std::endl;

	s_mm(2, 4, 3, alpha, m9a, m9b, result);
	if (!s_is_equal(8, o9, result, error))std::cout << "\"s_mm\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mm(2, 4, 3, alpha, m10a, 6, m10b, 6, result, 5);
	if (!s_is_equal(10, o10, result, error))std::cout << "\"s_mm with ld\" failed" << std::endl;

	s_mm(2, 4, 3, m11a, ColMajor{2}, m11b, 4, result, 4);
	if (!s_is_equal(8, o11, result, error))std::cout << "\"s_mmTN\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mm(2, 4, 3, m12a, ColMajor{ 5 }, m12b, 6, result, 5);
	if (!s_is_equal(10, o12, result, error))std::cout << "\"s_mmTN with ld\" failed" << std::endl;

	s_mm(2, 4, 3, alpha, m13a, ColMajor{2}, m13b, 4, result, 4);
	if (!s_is_equal(8, o13, result, error))std::cout << "\"s_mmTN\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mm(2, 4, 3, alpha, m14a, ColMajor{ 5 }, m14b, 6, result, 5);
	if (!s_is_equal(10, o14, result, error))std::cout << "\"s_mmTN with ld\" failed" << std::endl;

	s_mm(2, 4, 3, m15a, 3, m15b, ColMajor{3}, result, 4);
	if (!s_is_equal(8, o15, result, error))std::cout << "\"s_mmNT\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mm(2, 4, 3, m16a, 6, m16b, ColMajor{ 5 }, result, 5);
	if (!s_is_equal(10, o16, result, error))std::cout << "\"s_mmNT with ld\" failed" << std::endl;

	s_mm(2, 4, 3, alpha, m17a, 3, m17b, ColMajor{ 3 }, result, 4);
	if (!s_is_equal(8, o17, result, error))std::cout << "\"s_mmNT\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mm(2, 4, 3, alpha, m18a, 6, m18b, ColMajor{ 5 }, result, 5);
	if (!s_is_equal(10, o18, result, error))std::cout << "\"s_mmNT with ld\" failed" << std::endl;

	s_mm(2, 4, 3, m19a, ColMajor{ 2 }, m19b, ColMajor{3}, result, 4);
	if (!s_is_equal(8, o19, result, error))std::cout << "\"s_mmTT\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mm(2, 4, 3, m20a, ColMajor{ 5 }, m20b, ColMajor{ 5 }, result, 5);
	if (!s_is_equal(10, o20, result, error))std::cout << "\"s_mmTT with ld\" failed" << std::endl;

	s_mm(2, 4, 3, alpha, m21a, ColMajor{ 2 }, m21b, ColMajor{3}, result, 4);
	if (!s_is_equal(8, o21, result, error))std::cout << "\"s_mmTT\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mm(2, 4, 3, alpha, m22a, ColMajor{ 5 }, m22b, ColMajor{ 5 }, result, 5);
	if (!s_is_equal(10, o22, result, error))std::cout << "\"s_mmTT with ld\" failed" << std::endl;

	s_mmi(2, 4, 3, m27a, m27b, result);
	if (!s_is_equal(8, o27, result, error))std::cout << "\"s_mmi\" failed" << std::endl;

	std::fill_n(result, 36, 0);
	s_mmi(2, 4, 3, m28a, 6, m28b, 6, result, 5);
	if (!s_is_equal(10, o28, result, error))std::cout << "\"s_mmi with ld\" failed" << std::endl;

	const double ma01a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double ma01b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double oa01[]{ 2.31858183661872,3.09529802045975,3.989581394872564,4.962509892352645,5.774959561865027,6.749377875701963,8.0047379842151,9.32916821737827 };
	double oa01_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma02a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double ma02b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double oa02[]{ 2.31858183661872,3.09529802045975,3.989581394872564,4.962509892352645,0,5.774959561865027,6.749377875701963,8.0047379842151,9.32916821737827,0 };
	double oa02_before[]{ 1,2,3,4,0,5,6,7,8,0 };
	const double ma03a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double ma03b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double oa03[]{ 1.336363785854748,2.27940517499149,3.252437380181361,4.245531572120387,5.197688398910224,6.191162635724814,7.256303752089665,8.339064319876901 };
	double oa03_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma04a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double ma04b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double oa04[]{ 1.336363785854748,2.27940517499149,3.252437380181361,4.245531572120387,0,5.197688398910224,6.191162635724814,7.256303752089665,8.339064319876901,0 };
	double oa04_before[]{ 1,2,3,4,0,5,6,7,8,0 };

	const double ma05a[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double ma05b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double oa05[]{ 2.31858183661872,3.09529802045975,3.989581394872564,4.962509892352645,5.774959561865027,6.749377875701963,8.0047379842151,9.32916821737827 };
	double oa05_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma06a[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	const double ma06b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double oa06[]{ 2.31858183661872,3.09529802045975,3.989581394872564,4.962509892352645,0,5.774959561865027,6.749377875701963,8.0047379842151,9.32916821737827,0 };
	double oa06_before[]{ 1,2,3,4,0,5,6,7,8,0 };
	const double ma07a[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double ma07b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double oa07[]{ 1.336363785854748,2.27940517499149,3.252437380181361,4.245531572120387,5.197688398910224,6.191162635724814,7.256303752089665,8.339064319876901 };
	double oa07_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma08a[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	const double ma08b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double oa08[]{ 1.336363785854748,2.27940517499149,3.252437380181361,4.245531572120387,0,5.197688398910224,6.191162635724814,7.256303752089665,8.339064319876901,0 };
	double oa08_before[]{ 1,2,3,4,0,5,6,7,8,0 };

	const double ma09a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double ma09b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0.913375856139019,0.63235924622541,0.0975404049994095,0.278498218867048,0.546881519204984,0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616 };
	const double oa09[]{ 2.31858183661872,3.09529802045975,3.989581394872564,4.962509892352645,5.774959561865027,6.749377875701963,8.0047379842151,9.32916821737827 };
	double oa09_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma10a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double ma10b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0,0,0.913375856139019,0.63235924622541,0.0975404049994095,0,0,0.278498218867048,0.546881519204984,0.957506835434298,0,0,0.964888535199277,0.157613081677548,0.970592781760616,0,0 };
	const double oa10[]{ 2.31858183661872,3.09529802045975,3.989581394872564,4.962509892352645,0,5.774959561865027,6.749377875701963,8.0047379842151,9.32916821737827,0 };
	double oa10_before[]{ 1,2,3,4,0,5,6,7,8,0 };
	const double ma11a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double ma11b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0.913375856139019,0.63235924622541,0.0975404049994095,0.278498218867048,0.546881519204984,0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616 };
	const double oa11[]{ 1.336363785854748,2.27940517499149,3.252437380181361,4.245531572120387,5.197688398910224,6.191162635724814,7.256303752089665,8.339064319876901 };
	double oa11_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma12a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double ma12b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0,0,0.913375856139019,0.63235924622541,0.0975404049994095,0,0,0.278498218867048,0.546881519204984,0.957506835434298,0,0,0.964888535199277,0.157613081677548,0.970592781760616,0,0 };
	const double oa12[]{ 1.336363785854748,2.27940517499149,3.252437380181361,4.245531572120387,0,5.197688398910224,6.191162635724814,7.256303752089665,8.339064319876901,0 };
	double oa12_before[]{ 1,2,3,4,0,5,6,7,8,0 };

	const double ma13a[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double ma13b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0.913375856139019,0.63235924622541,0.0975404049994095,0.278498218867048,0.546881519204984,0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616 };
	const double oa13[]{ 2.31858183661872,3.09529802045975,3.989581394872564,4.962509892352645,5.774959561865027,6.749377875701963,8.0047379842151,9.32916821737827 };
	double oa13_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma14a[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	const double ma14b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0,0,0.913375856139019,0.63235924622541,0.0975404049994095,0,0,0.278498218867048,0.546881519204984,0.957506835434298,0,0,0.964888535199277,0.157613081677548,0.970592781760616,0,0 };
	const double oa14[]{ 2.31858183661872,3.09529802045975,3.989581394872564,4.962509892352645,0,5.774959561865027,6.749377875701963,8.0047379842151,9.32916821737827,0 };
	double oa14_before[]{ 1,2,3,4,0,5,6,7,8,0 };
	const double ma15a[]{ 0.498364051982143,0.585267750979777,0.959743958516081,0.223811939491137,0.340385726666133,0.751267059305653 };
	const double ma15b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0.913375856139019,0.63235924622541,0.0975404049994095,0.278498218867048,0.546881519204984,0.957506835434298,0.964888535199277,0.157613081677548,0.970592781760616 };
	const double oa15[]{ 1.336363785854748,2.27940517499149,3.252437380181361,4.245531572120387,5.197688398910224,6.191162635724814,7.256303752089665,8.339064319876901 };
	double oa15_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma16a[]{ 0.498364051982143,0.585267750979777,0,0,0,0.959743958516081,0.223811939491137,0,0,0,0.340385726666133,0.751267059305653,0,0,0 };
	const double ma16b[]{ 0.814723686393179,0.905791937075619,0.126986816293506,0,0,0.913375856139019,0.63235924622541,0.0975404049994095,0,0,0.278498218867048,0.546881519204984,0.957506835434298,0,0,0.964888535199277,0.157613081677548,0.970592781760616,0,0 };
	const double oa16[]{ 1.336363785854748,2.27940517499149,3.252437380181361,4.245531572120387,0,5.197688398910224,6.191162635724814,7.256303752089665,8.339064319876901,0 };
	double oa16_before[]{ 1,2,3,4,0,5,6,7,8,0 };

	const double ma17a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0.585267750979777,0.223811939491137,0.751267059305653 };
	const double ma17b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616 };
	const double oa17[]{ -0.31858183661872,0.90470197954025,2.01041860512744,3.03749010764736,4.22504043813497,5.25062212429804,5.9952620157849,6.67083178262173 };
	double oa17_before[]{ 1,2,3,4,5,6,7,8 };
	const double ma18a[]{ 0.498364051982143,0.959743958516081,0.340385726666133,0,0,0,0.585267750979777,0.223811939491137,0.751267059305653,0,0,0 };
	const double ma18b[]{ 0.814723686393179,0.913375856139019,0.278498218867048,0.964888535199277,0,0,0.905791937075619,0.63235924622541,0.546881519204984,0.157613081677548,0,0,0.126986816293506,0.0975404049994095,0.957506835434298,0.970592781760616,0,0 };
	const double oa18[]{ -0.31858183661872,0.90470197954025,2.01041860512744,3.03749010764736,0,4.22504043813497,5.25062212429804,5.9952620157849,6.67083178262173,0 };
	double oa18_before[]{ 1,2,3,4,0,5,6,7,8,0 };

	s_mma(2, 4, 3, ma01a, ma01b, oa01_before);
	if (!s_is_equal(8, oa01_before, oa01, error))std::cout << "\"s_mma\" failed" << std::endl;

	s_mma(2, 4, 3, ma02a, 6, ma02b, 6, oa02_before, 5);
	if (!s_is_equal(10, oa02_before, oa02, error))std::cout << "\"s_mma with ld\" failed" << std::endl;

	s_mma(2, 4, 3, alpha, ma03a, ma03b, oa03_before);
	if (!s_is_equal(8, oa03_before, oa03, error))std::cout << "\"s_mma\" failed" << std::endl;

	s_mma(2, 4, 3, alpha, ma04a, 6, ma04b, 6, oa04_before, 5);
	if (!s_is_equal(10, oa04_before, oa04, error))std::cout << "\"s_mma with ld\" failed" << std::endl;

	s_mma(2, 4, 3, ma06a, ColMajor{ 5 }, ma06b, RowMajor{ 6 }, oa06_before, RowMajor{ 5 });
	if (!s_is_equal(10, oa06_before, oa06, error))std::cout << "\"s_mmaTN with ld\" failed" << std::endl;

	s_mma(2, 4, 3, alpha, ma08a, ColMajor{ 5 }, ma08b, RowMajor{ 6 }, oa08_before, RowMajor{ 5 });
	if (!s_is_equal(10, oa08_before, oa08, error))std::cout << "\"s_mmaTN with ld\" failed" << std::endl;

	s_mma(2, 4, 3, ma10a, RowMajor{ 6 }, ma10b, ColMajor{ 5 }, oa10_before, RowMajor{ 5 });
	if (!s_is_equal(10, oa06_before, oa10, error))std::cout << "\"s_mmaNT with ld\" failed" << std::endl;

	s_mma(2, 4, 3, alpha, ma12a, RowMajor{ 6 }, ma12b, ColMajor{ 5 }, oa12_before, RowMajor{ 5 });
	if (!s_is_equal(10, oa12_before, oa12, error))std::cout << "\"s_mmaNT with ld\" failed" << std::endl;

	s_mma(2, 4, 3, ma14a, ColMajor{ 5 }, ma14b, ColMajor{ 5 }, oa14_before, RowMajor{ 5 });
	if (!s_is_equal(10, oa14_before, oa14, error))std::cout << "\"s_mmaTT with ld\" failed" << std::endl;

	s_mma(2, 4, 3, alpha, ma16a, ColMajor{ 5 }, ma16b, ColMajor{ 5 }, oa16_before, RowMajor{ 5 });
	if (!s_is_equal(10, oa16_before, oa16, error))std::cout << "\"s_mmaTT with ld\" failed" << std::endl;

	s_mms(2, 4, 3, ma17a, ma17b, oa17_before);
	if (!s_is_equal(8, oa17_before, oa17, error))std::cout << "\"s_mms\" failed" << std::endl;

	s_mms(2, 4, 3, ma18a, 6, ma18b, 6, oa18_before, 5);
	if (!s_is_equal(10, oa18_before, oa18, error))std::cout << "\"s_mms with ld\" failed" << std::endl;

	const double v01[]{ 0.3500,0.1966,0.2511,0.6160,0.4733,0.3517,0.8308,0.5853,0.5497,0.9172 };
	const double v02[]{ 0.8308,0.3517,0.3500,0.6160,0.5853,0.4733,0.2511,0.1966,0.5497,0.9172 };
	aris::Size p[10]{ 6,5,0,3,7,4,2,1,8,9 };

	s_vc(10, v01, result);
	s_permutate(10, 1, p, result, 1);
	if (!s_is_equal(10, v02, result, error))std::cout << "\"s_permutate\" failed" << std::endl;

	s_vc(10, v02, result);
	s_permutate_inv(10, 1, p, result, 1);
	if (!s_is_equal(10, v01, result, error))std::cout << "\"s_permutate_inv\" failed" << std::endl;
}
void test_llt(){
	const double A[]{ 1.82553083943141,1.42060601118548,1.36736238745112,1.50658906468564,1.86464891726001,1.04079482779702,
		1.42060601118548,2.10941693872417,1.92463386848915,1.23889223270807,2.23186828169132,1.22211204078486,
		1.36736238745112,1.92463386848915,2.06653199450749,1.37659815598197,2.07988145626914,1.30113287432829,
		1.50658906468564,1.23889223270807,1.37659815598197,1.69212820994619,1.67619205287543,0.914095057763804,
		1.86464891726001,2.23186828169132,2.07988145626914,1.67619205287543, 3.0881251584706,1.69495025317372,
		1.04079482779702,1.22211204078486,1.30113287432829,0.914095057763804,1.69495025317372,1.17872570447206 };

	const double A_ld[]{ 1.82553083943141,1.42060601118548,1.36736238745112,1.50658906468564,1.86464891726001,1.04079482779702,0.0,
		1.42060601118548,2.10941693872417,1.92463386848915,1.23889223270807,2.23186828169132,1.22211204078486,0.0,
		1.36736238745112,1.92463386848915,2.06653199450749,1.37659815598197,2.07988145626914,1.30113287432829,0.0,
		1.50658906468564,1.23889223270807,1.37659815598197,1.69212820994619,1.67619205287543,0.914095057763804,0.0,
		1.86464891726001,2.23186828169132,2.07988145626914,1.67619205287543, 3.0881251584706,1.69495025317372,0.0,
		1.04079482779702,1.22211204078486,1.30113287432829,0.914095057763804,1.69495025317372,1.17872570447206,0.0 };

	const double b[]{ 0.765516788149002,0.709364830858073,
		0.795199901137063,0.754686681982361,
		0.186872604554379,0.276025076998578,
		0.489764395788231,0.679702676853675,
		0.445586200710899,0.655098003973841,
		0.646313010111265,0.162611735194631 };

	const double bT[]{ 0.765516788149002,0.795199901137063,0.186872604554379,0.489764395788231,0.445586200710899,0.646313010111265,
		0.709364830858073,0.754686681982361,0.276025076998578,0.679702676853675,0.655098003973841,0.162611735194631 };

	const double llt[]{ 1.35112206681388,1.05142684445637,1.01201987669074,1.11506510158506,1.38007435675823,0.770318873002606,
		1.05142684445637,1.00195734913254,0.858887859606139,0.0663529756457487,0.779295701883798,0.411372898532454,
		1.01201987669074,0.858887859606139,0.551959607499983,0.346294079395723,0.025166983849247,0.30478983036659,
		1.11506510158506,0.0663529756457487,0.346294079395723,0.569592593325883,0.135000937041167, -0.136419197085494,
		1.38007435675823,0.779295701883798,0.025166983849247,0.135000937041167,0.746565139312267,0.431332592509531,
		0.770318873002606,0.411372898532454,0.30478983036659,-0.136419197085494,0.431332592509531,0.344313858756401 };

	const double llt_ld[]{ 1.35112206681388,1.05142684445637,1.01201987669074,1.11506510158506,1.38007435675823,0.770318873002606,0.0,
		1.05142684445637,1.00195734913254,0.858887859606139,0.0663529756457487,0.779295701883798,0.411372898532454,0.0,
		1.01201987669074,0.858887859606139,0.551959607499983,0.346294079395723,0.025166983849247,0.30478983036659,0.0,
		1.11506510158506,0.0663529756457487,0.346294079395723,0.569592593325883,0.135000937041167, -0.136419197085494,0.0,
		1.38007435675823,0.779295701883798,0.025166983849247,0.135000937041167,0.746565139312267,0.431332592509531,0.0,
		0.770318873002606,0.411372898532454,0.30478983036659,-0.136419197085494,0.431332592509531,0.344313858756401,0.0 };

	const double inv_l[]{ 0.740125577519527,0,0,0,0,0,
		-0.77666769064225,0.998046474598712,0,0,0,0,
		-0.148473446503069, -1.55303031002242,1.81172677567722,0,0,0,
		-1.26816697217082,0.827928336123375, -1.10147193494276,1.75564080663506,0,0,
		-0.323125426175609, -1.13916294185447,0.138104552985304, -0.31747149916724,1.3394678472681,0,
		-0.69415520561586,1.93742418585395, -2.21317495390761,1.09330166190531, -1.67799269315532,2.90432689410723 };

	const double inv_l_dot_b[]{ 0.566578554929674,0.525019055110873,
		0.19909430193153,0.202271637373723,
		-1.01006636357342,-0.777291110197726,
		0.341580853217506,0.614513319403422,
		-0.686050087570733,-0.389108108915027,
		2.26054438577134,0.474995191389852 };
	const double inv_u_dot_b[]{ -1.29249793785305,-1.28863719172842,
		1.65350463041262,0.456066717040007,
		-1.56976591868037,-0.518007602446926,
		1.42500452780045,1.16312249077394,
		-0.487660119419236,0.604621409654585,
		1.87710425727744,0.472277635823185 };


	const double B[]{ 0.854463601335834,0.915158806735392,0.81779406393944,0,0,0,
		0.915158806735392,1.51973084603713,1.08911106493401,0,0,0,
		0.81779406393944,1.08911106493401,1.00718816511107,0,0,0,
		0,0,0,1.16505945742745,0.55867090856838,1.06119850911405,
		0,0,0,0.55867090856838,0.321956437380287,0.442097312015519,
		0,0,0,1.06119850911405,0.442097312015519,1.05023839402297 };

	const double B_llt[]{ 0.924372003760301,0.990033020269512,0.88470232829715,0,0,0,
		0.990033020269512,0.734551199585949,0.290281394858118,0,0,0,
		0.88470232829715,0.290281394858118,0.374468512983265,0,0,0,
		0,0,0,1.07937920001613,0.517585394048759,0.983156344960322,
		0,0,0,0.517585394048759,0.232511929258862, -0.287168286010691,
		0,0,0,0.983156344960322, -0.287168286010691,0.0342982637594407 };

	const double C[]{ 1.09756159899368,0.494892128610864,0,0,0,0.599982575914901,
		0.494892128610864,0.564212545623307,0,0,0,0.200848874880563,
		0,0,0.979041348469538,0.638782936554467,0.663380272499309,0,
		0,0,0.638782936554467,0.791575910209436,0.454993751889092,0,
		0,0,0.663380272499309,0.454993751889092,0.470703623796929,0,
		0.599982575914901,0.200848874880563,0,0,0,1.00673482446645 };

	const double C_llt[]{ 1.04764574117097,0.472384995387576,0,0,0,0.572696048231236,
		0.472384995387576,0.584007672685888,0,0,0, -0.119320598822187,
		0,0,0.989465183050691,0.645584046307713,0.670443269619648,0,
		0,0,0.645584046307713,0.612206786439351,0.0362071665314066,0,
		0,0,0.670443269619648,0.0362071665314066,0.141061997399768,0,
		0.572696048231236, -0.119320598822187,0,0,0,0.815178910119423 };
	double result[100];

	s_llt(6, A, result);
	if (!s_is_equal(6, 6, llt, result, error))std::cout << "\"s_llt\" failed" << std::endl;

	s_llt(6, A_ld, 7, result, ColMajor{ 8 });
	if (!s_is_equal(6, 6, llt, 6, result, ColMajor{ 8 }, error))std::cout << "\"s_llt\" failed" << std::endl;

	s_mc(6, 6, A, result);
	s_llt(6, result, result);
	if (!s_is_equal(6, 6, llt, result, error))std::cout << "\"s_llt\" failed" << std::endl;

	s_mc(6, 6, A_ld, 7, result, ColMajor{ 8 });
	s_llt(6, result, ColMajor{ 8 }, result, ColMajor{ 8 });
	if (!s_is_equal(6, 6, llt, 6, result, ColMajor{ 8 }, error))std::cout << "\"s_llt\" failed" << std::endl;

	s_inv_lm(6, llt, result);
	if (!s_is_equal(6, 6, inv_l, result, error))std::cout << "\"s_inv_lm\" failed" << std::endl;

	s_mc(6, 6, llt, result);
	s_inv_lm(6, result, result);
	if (!s_is_equal(6, 6, inv_l, result, error))std::cout << "\"s_inv_lm\" failed" << std::endl;

	s_inv_lm(6, llt_ld, 7, result, ColMajor{ 8 });
	if (!s_is_equal(6, 6, inv_l, 6, result, ColMajor{ 8 }, error))std::cout << "\"s_inv_lm\" failed" << std::endl;

	s_mc(6, 6, llt, 6, result, ColMajor{ 8 });
	s_inv_lm(6, result, ColMajor{ 8 }, result, ColMajor{ 8 });
	if (!s_is_equal(6, 6, inv_l, 6, result, ColMajor{ 8 }, error))std::cout << "\"s_inv_lm\" failed" << std::endl;

	s_inv_um(6, llt, result);
	if (!s_is_equal(6, 6, inv_l, ColMajor{ 6 }, result, 6, error))std::cout << "\"s_inv_um\" failed" << std::endl;

	s_mc(6, 6, llt, result);
	s_inv_um(6, result, result);
	if (!s_is_equal(6, 6, inv_l, ColMajor{ 6 }, result, 6, error))std::cout << "\"s_inv_um\" failed" << std::endl;

	s_inv_um(6, llt_ld, 7, result, ColMajor{ 8 });
	if (!s_is_equal(6, 6, inv_l, ColMajor{ 6 }, result, ColMajor{ 8 }, error))std::cout << "\"s_inv_um\" failed" << std::endl;

	s_mc(6, 6, llt, 6, result, ColMajor{ 8 });
	s_inv_um(6, result, ColMajor{ 8 }, result, ColMajor{ 8 });
	if (!s_is_equal(6, 6, inv_l, ColMajor{ 6 }, result, ColMajor{ 8 }, error))std::cout << "\"s_inv_um\" failed" << std::endl;

	s_sov_lm(6, 2, llt, b, result);
	if (!s_is_equal(6, 2, inv_l_dot_b, result, error))std::cout << "\"s_sov_lm\" failed" << std::endl;

	s_mc(6, 2, b, result);
	s_sov_lm(6, 2, llt, result, result);
	if (!s_is_equal(6, 2, inv_l_dot_b, result, error))std::cout << "\"s_sov_lm\" failed" << std::endl;

	s_sov_lm(6, 2, llt, 6, bT, ColMajor{ 6 }, result, 2);
	if (!s_is_equal(6, 2, inv_l_dot_b, result, error))std::cout << "\"s_sov_lmNT\" failed" << std::endl;

	s_mc(6, 2, bT, ColMajor{ 6 }, result, ColMajor{ 6 });
	s_sov_lm(6, 2, llt, 6, result, ColMajor{ 6 }, result, ColMajor{ 6 });
	if (!s_is_equal(6, 2, inv_l_dot_b, 2, result, ColMajor{ 6 }, error))std::cout << "\"s_sov_lmNT\" failed" << std::endl;

	s_sov_um(6, 2, llt, b, result);
	if (!s_is_equal(6, 2, inv_u_dot_b, result, error))std::cout << "\"s_sov_um\" failed" << std::endl;

	s_mc(6, 2, b, result);
	s_sov_um(6, 2, llt, result, result);
	if (!s_is_equal(6, 2, inv_u_dot_b, result, error))std::cout << "\"s_sov_um\" failed" << std::endl;

	s_sov_um(6, 2, llt, 6, bT, ColMajor{ 6 }, result, 2);
	if (!s_is_equal(6, 2, inv_u_dot_b, result, error))std::cout << "\"s_sov_lmNT\" failed" << std::endl;

	s_mc(6, 2, bT, ColMajor{ 6 }, result, ColMajor{ 6 });
	s_sov_um(6, 2, llt, 6, result, ColMajor{ 6 }, result, ColMajor{ 6 });
	if (!s_is_equal(6, 2, inv_u_dot_b, 2, result, ColMajor{ 6 }, error))std::cout << "\"s_sov_umNT\" failed" << std::endl;
}
void test_householder(){
	using aris::Size;

	auto check_household = [](Size m, Size n, Size rhs
		, const double *A, const double *U, const double *tau, const double *Q, const double *R, const double *x, const double *b, const double *ss, const double *pinv
		, const double *U_p, const double *tau_p, const Size *p, const double *x_p
		, Size a_t, Size u_t, Size tau_t, Size q_t, Size r_t, Size x_t, Size b_t, Size ss_t, Size pinv_t)->void
	{
		auto max_size = std::max({ m, n, rhs, a_t, u_t, tau_t, q_t, r_t, x_t, b_t, ss_t });
		std::vector<double> input1(max_size * max_size), input2(max_size * max_size), input3(max_size * max_size);
		std::vector<double> result1(max_size * max_size), result2(max_size * max_size), result3(max_size * max_size);
		std::vector<Size> resultp(max_size);

		auto i1_ = input1.data();
		auto i2_ = input2.data();
		auto i3_ = input3.data();
		auto r1_ = result1.data();
		auto r2_ = result2.data();
		auto r3_ = result3.data();
		auto rp_ = resultp.data();

		s_householder_ut(m, n, A, r1_, r2_);
		if (!(s_is_equal(m, n, r1_, U, error) && s_is_equal(std::min({ m - 1, m, n }), r2_, tau, error)))std::cout << "\"s_householder_ut\" failed" << std::endl;

		s_mc(m, n, A, n, i1_, a_t);
		s_householder_ut(m, n, i1_, a_t, r1_, u_t, r2_, tau_t);
		if (!(s_is_equal(m, n, r1_, u_t, U, n, error) && s_is_equal(std::min({ m - 1, m, n }), 1, r2_, tau_t, tau, 1, error)))std::cout << "\"s_householder_ut\" failed" << std::endl;

		s_mc(m, n, A, n, r1_, u_t);
		s_householder_ut(m, n, i1_, a_t, r1_, u_t, r2_, tau_t);
		if (!(s_is_equal(m, n, r1_, u_t, U, n, error) && s_is_equal(std::min({ m - 1, m, n }), 1, r2_, tau_t, tau, 1, error)))std::cout << "\"s_householder_ut\" failed" << std::endl;

		s_householder_ut2q(m, n, U, tau, r1_);
		if (!s_is_equal(m, m, r1_, Q, error))std::cout << "\"s_householder_ut2q\" failed" << std::endl;

		s_mc(m, n, U, n, i1_, u_t);
		s_mc(m, 1, tau, 1, i2_, tau_t);
		s_householder_ut2q(m, n, i1_, u_t, i2_, tau_t, r1_, q_t);
		if (!s_is_equal(m, m, r1_, q_t, Q, m, error))std::cout << "\"s_householder_ut2q\" failed" << std::endl;

		s_householder_ut2qmn(m, n, U, tau, r1_);
		if (!s_is_equal(m, std::min(m, n), r1_, n, Q, m, error))std::cout << "\"s_householder_ut2q\" failed" << std::endl;

		s_mc(m, n, U, n, i1_, u_t);
		s_mc(m, 1, tau, 1, i2_, tau_t);
		s_householder_ut2qmn(m, n, i1_, u_t, i2_, tau_t, r1_, q_t);
		if (!s_is_equal(m, std::min(m, n), r1_, q_t, Q, m, error))std::cout << "\"s_householder_ut2q\" failed" << std::endl;

		s_householder_ut2r(m, n, U, tau, r1_);
		if (!s_is_equal(m, n, r1_, R, error))std::cout << "\"s_householder_ut2r\" failed" << std::endl;

		s_mc(m, n, U, n, i1_, u_t);
		s_mc(m, 1, tau, 1, i2_, tau_t);
		s_householder_ut2r(m, n, i1_, u_t, i2_, tau_t, r1_, r_t);
		if (!s_is_equal(m, n, r1_, r_t, R, n, error))std::cout << "\"s_householder_ut2r\" failed" << std::endl;

		s_householder_ut2qr(m, n, U, tau, r1_, r2_);
		if (!(s_is_equal(m, m, r1_, Q, error) && s_is_equal(m, n, r2_, R, error)))std::cout << "\"s_householder_ut2qr\" failed" << std::endl;

		s_mc(m, n, U, n, i1_, u_t);
		s_mc(m, 1, tau, 1, i2_, tau_t);
		s_householder_ut2qr(m, n, i1_, u_t, i2_, tau_t, r1_, q_t, r2_, r_t);
		if (!(s_is_equal(m, m, r1_, q_t, Q, m, error) && s_is_equal(m, n, r2_, r_t, R, n, error)))std::cout << "\"s_householder_ut2qr\" failed" << std::endl;

		s_householder_ut_sov(m, n, rhs, U, tau, b, r1_);
		if (!(s_is_equal(n, rhs, r1_, x, error)))std::cout << "\"s_householder_ut_sov\" failed" << std::endl;

		s_mc(m, n, U, n, i1_, u_t);
		s_mc(m, 1, tau, 1, i2_, tau_t);
		s_mc(m, rhs, b, rhs, i3_, b_t);
		s_householder_ut_sov(m, n, rhs, i1_, u_t, i2_, tau_t, i3_, b_t, r1_, x_t);
		if (!(s_is_equal(n, rhs, r1_, x_t, x, rhs, error)))std::cout << "\"s_householder_ut_sov\" failed" << std::endl;

		Size rank;

		s_householder_up(m, n, A, r1_, rp_, rank);
		if (!(s_is_equal(m, n, r1_, U_p, error) && std::equal(p, p + std::min(m, n), rp_)))std::cout << "\"s_householder_up\" failed" << std::endl;

		s_mc(m, n, A, n, i1_, a_t);
		s_householder_up(m, n, i1_, a_t, r1_, u_t, rp_, rank);
		if (!(s_is_equal(m, n, r1_, u_t, U_p, n, error) && std::equal(rp_, rp_ + std::min(m, n), p)))std::cout << "\"s_householder_up ld\" failed" << std::endl;

		s_householder_up_sov(m, n, rhs, rank, U_p, p, b, r1_);
		if (!(s_is_equal(n, rhs, r1_, x_p, error)))std::cout << "\"s_householder_up_sov\" failed" << std::endl;

		s_mc(m, n, U_p, n, i1_, u_t);
		s_mc(m, rhs, b, rhs, i3_, b_t);
		s_householder_up_sov(m, n, rhs, rank, i1_, u_t, p, i3_, b_t, r1_, x_t);
		if (!(s_is_equal(n, rhs, r1_, x_t, x_p, rhs, error)))std::cout << "\"s_householder_up_sov\" failed" << std::endl;



		s_householder_up2pinv(m, n, rank, U_p, tau_p, p, r1_, r2_);
		aris::dynamic::dsp(n, m, r1_);
		aris::dynamic::dsp(n, m, pinv);
		if (!(s_is_equal(n, m, r1_, pinv, error)))std::cout << "\"s_householder_up2pinv\" failed" << std::endl;












		s_householder_utp(m, n, A, r1_, r2_, rp_, rank);
		if (!(s_is_equal(m, n, r1_, U_p, error) && s_is_equal(std::min({ m - 1, m, n }), r2_, tau_p, error) && std::equal(p, p + std::min(m, n), rp_)))std::cout << "\"s_householder_utp\" failed" << std::endl;

		s_mc(m, n, A, n, i1_, a_t);
		s_householder_utp(m, n, i1_, a_t, r1_, u_t, r2_, tau_t, rp_, rank);
		if (!(s_is_equal(m, n, r1_, u_t, U_p, n, error) && s_is_equal(std::min({ m - 1, m, n }), 1, r2_, tau_t, tau_p, 1, error) && std::equal(rp_, rp_ + std::min(m, n), p)))std::cout << "\"s_householder_utp ld\" failed" << std::endl;

		s_householder_utp_sov(m, n, rhs, rank, U_p, tau_p, p, b, r1_);
		if (!(s_is_equal(n, rhs, r1_, x_p, error)))std::cout << "\"s_householder_utp_sov\" failed" << std::endl;

		s_mc(m, n, U_p, n, i1_, u_t);
		s_mc(m, 1, tau_p, 1, i2_, tau_t);
		s_mc(m, rhs, b, rhs, i3_, b_t);
		s_householder_utp_sov(m, n, rhs, rank, i1_, u_t, i2_, tau_t, p, i3_, b_t, r1_, x_t);
		if (!(s_is_equal(n, rhs, r1_, x_t, x_p, rhs, error)))std::cout << "\"s_householder_utp_sov\" failed" << std::endl;

		s_householder_utp_sov_solution_space(m, n, rank, U_p, tau_p, p, r1_);
		if (!(s_is_equal(n, n - rank, r1_, ss, error)))std::cout << "\"s_householder_utp_sov_solution_space\" failed" << std::endl;

		s_mc(m, n, U_p, n, i1_, u_t);
		s_mc(m, 1, tau_p, 1, i2_, tau_t);
		s_mc(m, rhs, b, rhs, i3_, b_t);
		s_householder_utp_sov_solution_space(m, n, rank, i1_, u_t, i2_, tau_t, p, r1_, ss_t);
		if (!(s_is_equal(n, n - rank, r1_, ss_t, ss, n - rank, error)))std::cout << "\"s_householder_utp_sov_solution_space ld\" failed" << std::endl;

		s_householder_utp2pinv(m, n, rank, U_p, tau_p, p, r1_, r2_);
		if (!(s_is_equal(n, m, r1_, pinv, error)))std::cout << "\"s_householder_utp2pinv\" failed" << std::endl;

		s_mc(m, n, U_p, n, i1_, u_t);
		s_mc(m, 1, tau_p, 1, i2_, tau_t);
		s_mc(m, rhs, b, rhs, i3_, b_t);
		s_householder_utp2pinv(m, n, rank, i1_, u_t, i2_, tau_t, p, r1_, pinv_t, r2_, 2);
		if (!(s_is_equal(n, m, r1_, pinv_t, pinv, m, error)))std::cout << "\"s_householder_utp2pinv ld\" failed" << std::endl;
	};
	
	{
		// test 5*6 mat
		const aris::Size m{ 5 }, n{ 6 }, rhs{ 2 };
		const aris::Size a_t{ 7 }, q_t{ 8 }, r_t{ 10 }, u_t{ 9 }, tau_t{ 2 }, b_t{ 4 }, x_t{ 5 }, ss_t{n+3}, pinv_t{12};
		const double A[]{ 
			0.8147,0.0975,0.1576,0.1419,0.6557,0.7577,
			0.9058,0.2785,0.9706,0.4218,0.0357,0.7431,
			0.1270,0.5469,0.9572,0.9157,0.8491,0.3922,
			0.9134,0.9575,0.4854,0.7922,0.9340,0.6555,
			0.6324,0.9649,0.8003,0.9595,0.6787,0.1712, };
		const double Q[]{
			-0.492666858742303, -0.480667841387474,0.177953454506114,0.656659986061627,0.25173006035788,
			-0.547757015648433, -0.358349168353002, -0.57774356601946, -0.475777233310347, -0.106754491874129,
			-0.0767996698910918,0.475432019801101, -0.634320532325754,0.554658862728443, -0.240950645401142,
			-0.552352901405695,0.339054939876482,0.480845521478708, -0.0718314601196047, -0.586153432016755,
			-0.382426072749027,0.547312015298238,0.0311446094260189, -0.172126332392726,0.723603756065892,};
		const double R[]{
			-1.65365294121832, - 1.14046790774039, - 1.25697758470928, - 1.1757905794716, - 1.18325736992819, - 1.23799141825475,
			0,0.966094882200634,0.634107648406748,1.0097398566765,0.763860337732694, - 0.128076522148321,
			0,0,-0.881556607241129, - 0.388478877543586,0.0277046342935185 ,- 0.222740227744911,
			0,0,0,0.178338236974704,0.700634820427496,0.284984865114035,
			0,0,0,0,-0.0997003640550825,-0.243436850953329,};
		const double U[]{
			-1.65365294121832, - 1.14046790774039, - 1.25697758470928, - 1.1757905794716, - 1.18325736992819, - 1.23799141825475,
			0.366965349595799,0.966094882200634,0.634107648406748,1.0097398566765,0.763860337732694, - 0.128076522148321,
			0.0514513130919259, - 0.423163816226815, - 0.881556607241129, - 0.388478877543586,0.0277046342935185, - 0.222740227744911,
			0.370044325812324, - 0.437343928487383, - 0.0698283162531727,0.178338236974704,0.700634820427497,0.284984865114035,
			0.256203231490819, - 0.567244456473452,0.197960951688692,0.489745227556259, - 0.0997003640550822, - 0.243436850953329};
		const double tau[]{
			-1.4926668587423,
			- 1.18196072589879,
			- 1.91559040502267,
			- 1.61309785397955};
		const double U_p[]{
			-1.6610921738422586, - 1.2513481868932035, - 1.1722856206683436, - 1.2318104812131321, - 1.0061279598555908, - 1.4813701062164377,
			0.5336801983094602,   1.0810623317640351,   0.4530354352986512,   0.3186812523342090,   0.7290900149725947,   0.0838432260611339,
			0.5263122664556104,   0.7515517997743093, - 0.9516736824868056, - 0.5243614642243333, - 0.0788369681589305, - 0.5276783338873579,
			0.2668950837208036, - 0.2832694383720395,   0.0273426548714987, - 0.5832123306486975,   0.3856056047995737, - 0.3228245505711993,
			0.4400414822862776,   0.2165686782664393,   0.0933261920358410,   0.4799795754951561, - 0.2008600469549645,   0.0251244372340313,
		};
		const double tau_p[]{
			-1.0948773358166253,
			- 1.1820514879811974,
			- 1.9812624110622559,
			- 1.6255135497552797,
			- 0.5051532609723628
		};
		const aris::Size p[]{ 2,0,4,1,5,3 };
		const double b[]{ 0.4387,0.4898,
			0.3816,0.4456,
			0.7655,0.6463,
			0.7952,0.7094,
			0.1869,0.7547 };
		const double x[]{ -2.75456661995493,0.868590789483354,
			7.99831100006118, - 1.88123572093792,
			7.05877107200108, - 1.35425833135642,
			- 15.0821489767352,3.59223425153636,
			4.4695817014224, - 0.504388227347691,
			0, 0};
		const double x_p[]{ -1.17295983130461,         0.491887039413251,
			0.897247863266881, -0.189919565550317,
			-0.208965837808077,         0.376755829745113,
			0,                         0,
			-0.136754011228341,          0.59273904188852,
			1.8865399618534, -0.449332086449719 };
		const double solution_space[]{ -0.104866142821558,
			0.470825685898474,
			0.481876748533644,
			-1,
			0.305416404502847,
			-0.125084294337861 };
		const double pinv[n*m]
		{
			1.68261057542629,-0.0902927747175428,  -1.18241528561313,  -1.74638034446082,   2.29172586736232
			,  -2.08466510617918,  0.314347790427471, -0.230988981636635,   2.63294696305148,  -1.47151413457989
			,  0.391675738206163,  0.430480921090843,  0.419438406462811,  -1.33551244134448,   0.77430279043145
			, -0.233389016413539,-0.0512834249044968,  0.234496874118209, 0.0135949139403609,  0.202029716378762
			,   1.70053861780348,  -1.01316566313889,   0.39520042337884,  -1.30232210440492,   1.09424065437846
			,  -1.73052838135578,  0.853475866630469,  0.827931426677011,   2.94119570286494,  -3.42059940559672
		};

		check_household(m, n, rhs, A, U, tau, Q, R, x, b, solution_space, pinv, U_p, tau_p, p, x_p, a_t, u_t, tau_t, q_t, r_t, x_t, b_t, ss_t, pinv_t);
	}
	{
	// test 6*5 mat	
		const aris::Size m{ 6 }, n{ 5 }, rhs{ 2 };
		const aris::Size a_t{ 9 }, q_t{ 8 }, r_t{ 6 }, u_t{ 7 }, tau_t{ 2 }, b_t{ 4 }, x_t{ 3 }, ss_t{1}, pinv_t{12};
		const double A[]{ 0.8147,0.9058, 0.127,0.9134,0.6324,
			0.0975,0.2785,0.5469,0.9575,0.9649,
			0.1576,0.9706,0.9572,0.4854,0.8003,
			0.1419,0.4218,0.9157,0.7922,0.9595,
			0.6557,0.0357,0.8491, 0.934,0.6787,
			0.7577,0.7431,0.3922,0.6555,0.1712 };
		const double Q[]{
			-0.6207938622228, - 0.154560758796689,0.479344322039396, - 0.321468254294596, 0.500625274984451, 0.0835910815985244,
			- 0.07429409790932, - 0.17707859766604, - 0.287176127497771, - 0.79778986158619, - 0.321450823922257, - 0.37530538713139,
			- 0.120089741851373, - 0.771426698803364, - 0.254652968468967, 0.386622567240651, 0.169145687493211, - 0.384114429341121,
			- 0.108126487111103, - 0.273077416783337, - 0.506838341291211, - 0.139647719197317, 0.0441331940078665, 0.797121733949575,
			- 0.499637333324524,   0.521984537055711, - 0.580180031408702, 0.157727038950421, 0.239035801052366, - 0.243454053933954,
			- 0.57736038959889, - 0.0511454590343965, 0.171513222014195, 0.257552294569938, - 0.747226595652759, 0.0997074095924538 };
		const double R[]{
			-1.31235189259588, - 1.19204640068418, - 0.984116392323211, - 1.62724008099373, - 1.10207955515584,
			0, - 1.07261967566136, - 0.681782306693105, - 0.447503294312055, - 0.80248312849391,
			0,0, - 1.22940896476355, - 0.79172515549949, - 1.02847417900777,
			0,0,0, - 0.664332641480675, - 0.646519613301525,
			0,0,0,0,0.218449022247435,
			0,0,0,0,0 };
		const double U[]{
			-1.31235189259588, - 1.19204640068418, - 0.984116392323211, - 1.62724008099373,-1.10207955515584,
			0.0458380918394095, - 1.07261967566136, - 0.681782306693105, - 0.447503294312055,-0.802483128493911,
			0.0740931617834968,0.64955454096634, - 1.22940896476355, - 0.79172515549949,- 1.02847417900777,
			0.0667120536616637,0.224587809775278,0.43088167987985, - 0.664332641480675,- 0.646519613301525,
			0.30826704429847, - 0.486866265436373,0.806399797389196,0.64673735967616,0.218449022247434,
			0.356220740376622, - 0.00334385430562029,0.000250527139268941, - 0.275874736646322,0.711610030647557 };
		const double tau[]{
			-1.6207938622228,
			-1.16999382740955,
			-1.08936038603983,
			-1.33835118423397,
			-1.32767845364868,
			0 };
		const double U_p[]{
			-1.9789734864317916, - 1.2227281146464213, - 1.4552228666754592, - 1.0791006623592800, - 1.7161598441238421,
			0.3310430013591469,   1.0375083361849231,   0.1205422679295597,   0.2360825672981153,   0.0733413570847640,
			0.1678206505062453, - 0.4190838645420051, - 0.9014180827713946,   0.3408882995178777, - 0.4325106805558034,
			0.2738927056675887,   0.1101281942121770,   0.3176468269001335,   0.6211839012446341, - 0.4434593675209600,
			0.3229181861821861,   0.4452182562066503,   0.2458013762676567, - 0.8994930705633606, - 0.2184490222474343,
			0.2266304829147998, - 0.1781245180361834,   0.0104789845315289, - 0.3076377256333895, - 0.7160073181950455,
		};
		const double tau_p[]{
			-1.4615524191013369,
			- 1.4107285895329176,
			- 1.7220186125457726,
			- 1.0505698333136935,
			- 1.3221685195172266,
		};
		const aris::Size p[]{ 3,1,2,0,4 };
		const double b[]{
			0.5060,0.1493,
			0.6991,0.2575,
			0.8909,0.8407,
			0.9593,0.2543,
			0.5472,0.8143,
			0.1386,0.2435
		};
		const double x[]{
			0.209748905164346, 0.947708395923203,
			0.072279163615098, -0.200871645791878,
			0.169848963487236, 0.699653165641312,
			-0.524722596591983, -1.04635860123482,
			1.13918462877824,0.723696504731286,
		};
		const double x_p[]{
			0.209748905164346, 0.947708395923203,
			0.072279163615098, -0.200871645791878,
			0.169848963487236, 0.699653165641312,
			-0.524722596591983, -1.04635860123482,
			1.13918462877824,0.723696504731286,
		};
		const double solution_space[1]{ 0.0 };
		const double pinv[n*m]
		{
			   1.68261057542629,  -2.08466510617918,  0.391675738206162,  -0.23338901641354,   1.70053861780348,  -1.73052838135578
			, -0.090292774717543,  0.314347790427471,  0.430480921090844,-0.0512834249044966,  -1.01316566313889,   0.85347586663047
			,  -1.18241528561313, -0.230988981636636,  0.419438406462811,   0.23449687411821,   0.39520042337884,   0.82793142667701
			,  -1.74638034446082,   2.63294696305148,  -1.33551244134448,  0.013594913940362,  -1.30232210440491,   2.94119570286495
			,   2.29172586736231,  -1.47151413457989,  0.774302790431449,  0.202029716378761,   1.09424065437846,  -3.42059940559672
		};

		check_household(m, n, rhs, A, U, tau, Q, R, x, b, solution_space, pinv, U_p, tau_p, p, x_p, a_t, u_t, tau_t, q_t, r_t, x_t, b_t, ss_t, pinv_t);
	}
	{
	// test 6*8 mat, rank = 3	
		const aris::Size m{ 6 }, n{ 8 }, rhs{ 2 };
		const aris::Size a_t{ 9 }, q_t{ 7 }, r_t{ 10 }, u_t{ 11 }, tau_t{ 3 }, b_t{ 4 }, x_t{ 3 }, ss_t{ 11 }, pinv_t{12};
		const double A[]{ 0.630229584015469,0.583502426070518,0.245043192948568,0.148989772647588,0.638168963369892,0.468971952494786,0.692188984164767,0.640647257722921,
			1.30421400810528,0.754614462691933,0.859748450111044,0.628092465488664,0.750030906805839, 1.23407805551515, 1.03970176314677,0.924448816182391,
			1.15742962510269,0.798001847460554, 0.63915040930505, 0.43488018328897, 0.90689215589439,0.994025053941913,0.928789589795695, 1.02781307174816,
			1.17630477409228,0.495900544836425,0.889366007731956,0.657024133959768,0.543202355039408, 1.18745586224613,0.646197013475914, 0.78716383805321,
			1.6221228942842,0.757633616862194, 1.16744478187744,0.851888888773582, 0.84715071263386, 1.59285803129098,0.948338702739599, 1.15689730325343,
			1.00698036729897,0.699439616042398,0.592134206691721,0.428199645433501,0.662125266341317,0.906189797762948,0.988761815716694,0.741776786151256 };
		const double Q[]{
			-0.21659046950618,0.59867574328494,-0.24833882313044,0.22680679774891,0.16440034899442,-0.67419207470343,
			- 0.44821812799115,0.04337995906672,0.44397955352511,-0.50079761437411,-0.52217549896072,-0.27683089181927,
			- 0.39777286290514,0.37721809878594,-0.53668223707494,-0.08018763396366,-0.30326292880043,0.55951494284556,
			- 0.40425966943619,-0.44844806488915,-0.03247150779567,0.71048663859794,-0.34540077264643,-0.10159279785025,
			- 0.55747360673108,-0.42363082469425,-0.24316200938271,-0.32209519817367,0.58438283153254,-0.07337389783535,
			- 0.34606809338775,0.34182348802851,0.62689712055526,0.28773909880074,0.38409879200055,0.37425647224271 };
		const double R[]{
			-2.90977523365818,-1.64692349537674,-1.90793876664392,-1.375475894873,-1.75613524978949,-2.73172740433524,-2.11746611364055,-2.18181253955987,
			0,0.378826928449942,-0.265898310479768,-0.22867103331949, 0.38054149143176,-0.188278458728485,0.456305923466501, 0.22180948762949,
			0,0,0.0362862782773966,0.0484233444675221,-0.120745699323159,0.0401726247370298,0.159513051359982,-0.142126370279255,
			0,0,0,1.61232079026924e-16,-6.48640105066524e-16,7.64711004481557e-17,6.56204014606395e-16,-8.04513232841518e-16,
			0,0,0,0,-4.50895641258737e-16,1.18921806382808e-17,3.40345183834875e-16,-5.20772154664907e-16,
			0,0,0,0,0,9.56084036058462e-17,9.75859929024559e-17,-6.30738539333844e-17 };
		const double U[]{
			-2.90977523365818,-1.64692349537674,-1.90793876664392,  -1.375475894873,-1.75613524978949,-2.73172740433524,-2.11746611364055,-2.18181253955987
			,0.368421534793942,0.378826928449942,-0.265898310479768,-0.22867103331949, 0.38054149143176,-0.188278458728485,0.456305923466501, 0.22180948762949
			,0.326957076251469,-0.154161679132974,0.0362862782773966,0.0484233444675221,-0.120745699323159,0.0401726247370298,0.159513051359982,-0.142126370279255
			,0.332289031986487,0.549940243622438,0.178033955607869,0.0,-6.48640105066524e-16,7.64711004481557e-17,6.56204014606395e-16,-8.04513232841518e-16
			,0.458226182683615, 0.59290568558141,0.325471378722472,-0.0,-4.50895641258737e-16,1.18921806382808e-17,3.40345183834875e-16,-5.20772154664907e-16
			,0.284457343750374,-0.145708418888381,-0.564892640444791,-0.0,-0.0,9.56084036058462e-17,9.75859929024559e-17,-6.30738539333844e-17 };
		const double tau[m]{
			-1.21659046950618, - 1.17718507711822, - 1.37293669646709,   0.00000000000000,   0.00000000000000 };
		const double U_p[m*n]{
			-2.90977523365818,-2.11746611364055,-1.75613524978949,-1.37547589487300,-1.90793876664393,-2.73172740433524,-1.64692349537674,-2.18181253955987,
			0.36842153479394,-0.48338339787873,-0.31937965268346,0.19988231292143,0.23902939084828,0.16447506963396,-0.35760634762987,-0.16248380153128,
			0.32695707625147,0.02080750329384,-0.23955790126386,0.12117063485022,0.12199817956804,0.10005283238538,-0.12501016700110,-0.20736047880721,
			0.33228903198649,-0.58903966304771,-0.39495676074010,-0.00000000000001,-0.00000000000000,-0.00000000000000,0.00000000000000,-0.00000000000000,
			0.45822618268361,-0.69499008831972,-0.33307292828213,0.00000000000000,0.00000000000000,0.00000000000000,0.00000000000000,0.00000000000000,
			0.28445734375037,0.38843257768054,-0.25493439927099,0.00000000000001,0.00000000000001,0.00000000000001,-0.00000000000000,0.00000000000000
		};
		const double tau_p[n]{
			-1.21659046950618, - 1.00944244215200, - 1.50159172513588,   0.00000000000000,   0.00000000000000
		};
		const aris::Size p[n]{ 0,6,4,3,2,5,1,7 };
		const double b[m*rhs]{
			0.5060,0.1493,
			0.6991,0.2575,
			0.8909,0.8407,
			0.9593,0.2543,
			0.5472,0.8143,
			0.1386,0.2435
		};
		const double x_p[n*rhs]{
			0.43966652206982,   0.32500288343441,
			0.00000000000000,   0.00000000000000,
			0.00000000000000,   0.00000000000000,
			0.00000000000000,  0.00000000000000,
			1.53618171824723,   1.74252653963440,
			0.00000000000000,   0.00000000000000,
			- 1.16126194227875, - 1.36135131463168,
			0.00000000000000,   0.00000000000000,
		};
		const double x[n*rhs]{
			11.19786472349186,   12.73040404664730,
			- 7.02668268744507, - 8.15974137482917,
			- 10.21663805982671, - 11.78285313579456,
			0.00000000000000,   0.00000000000000,
			0.00000000000000,   0.00000000000000,
			0.00000000000000,   0.00000000000000,
			0.00000000000000,   0.00000000000000,
			0.00000000000000 ,  0.00000000000000,
		};
		const double solution_space[n*n]{ 0.83569396555748,1.07804279292076,1.23767385623591,-0.03640050906400,0.39898452405905,
			0.00000000000000,0.00000000000000,0.00000000000000,-1.00000000000000,0.00000000000000,
			0.00000000000000,-1.00000000000000,0.00000000000000,0.00000000000000,0.00000000000000,
			-1.00000000000000,0.00000000000000,0.00000000000000,0.00000000000000,0.00000000000000,
			-0.50580938558464,-0.50926385197233,-0.41765615685194,0.52183696025709,0.86559649134183,
			0.00000000000000,0.00000000000000,-1.00000000000000,0.00000000000000,0.00000000000000,
			-0.07930989603203,-0.15801303689019,-0.06430545909083,0.39501199533020,-0.23577579564718,
			0.00000000000000,0.00000000000000,0.00000000000000,0.00000000000000,-1.00000000000000 };
		const double pinv[n*m]
		{
			-0.0470968191356404,   -0.04562751034156,  0.0587776043539224,   0.122335993764335,   0.167571020045078,  -0.140402541126087
		,   0.247125799673244,   0.133627982997883,   0.076257474475392,  -0.220126028877963,  -0.251592832944578,   0.320934588340986
		,  -0.346877584025024,   0.112345143400708,  -0.287565572844174,   0.249955287062167,   0.199672496084001,-0.00486262925338568
		,  -0.352772995924887,    0.22138335923286,  -0.393080270265157,    0.18797198979702,  0.0836654730517423,   0.183346841061631
		,   0.658647326647929,  -0.713178647219682,    1.06339623987355,  -0.105811160564627,   0.263473639413296,  -0.907876680313713
		,  -0.317376796508276,   0.159322279300896,  -0.293811635634574,   0.221726091712921,    0.15970417003719,  0.0768678750741678
		,  -0.248840714965721,    1.27623996673341,   -1.20634932053306,  -0.406207315055435,  -0.969548900128278,    1.99889492179419
		,   0.610002530571062,  -0.936436098824207,    1.22835137951625,  0.0529942518227028,   0.523516264862582,   -1.31316435069964
		};

		check_household(m, n, rhs, A, U, tau, Q, R, x, b, solution_space, pinv, U_p, tau_p, p, x_p, a_t, u_t, tau_t, q_t, r_t, x_t, b_t, ss_t, pinv_t);
	}
	{
		// test 8*6 mat, rank = 3	
		const aris::Size m{ 8 }, n{ 6 }, rhs{ 2 };
		const aris::Size a_t{ 9 }, q_t{ 12 }, r_t{ 10 }, u_t{ 11 }, tau_t{ 3 }, b_t{ 4 }, x_t{ 3 }, ss_t{ 11 }, pinv_t{12};
		const double A[]{ 0.630229584015469, 1.30421400810528, 1.15742962510269, 1.17630477409228,  1.6221228942842, 1.00698036729897,
			0.583502426070518,0.754614462691933,0.798001847460554,0.495900544836425,0.757633616862194,0.699439616042398,
			0.245043192948568,0.859748450111044,0.63915040930505,0.889366007731956, 1.16744478187744,0.592134206691721,
			0.148989772647588,0.628092465488664,0.43488018328897,0.657024133959768,0.851888888773582,0.428199645433501,
			0.638168963369892,0.750030906805839,0.90689215589439,0.543202355039408, 0.84715071263386,0.662125266341317,
			0.468971952494786,1.23407805551515,0.994025053941913, 1.18745586224613, 1.59285803129098,0.906189797762948,
			0.692188984164767,1.03970176314677,0.928789589795695,0.646197013475914,0.948338702739599,0.988761815716694,
			0.640647257722921,0.924448816182391,1.02781307174816, 0.78716383805321, 1.15689730325343,0.741776786151256 };
		const double Q[]{
			-0.41228854678567, 0.27203983451236,  0.0958966914655,-0.25712089887792,-0.36999550987336, -0.5015386923088, 0.03569581008172,-0.53943787639297
			,-0.38172020703589,-0.25586032951048,-0.13933216668124,  0.2836358995821,-0.32148387464018, 0.27580894786422,-0.70931077473034,-0.08011346076179
			,-0.16030428352283, 0.50111537399385, 0.05921028203466,-0.23498172837939, 0.39107004722063,-0.24607632624617,-0.52088900111188, 0.42385015843946
			,-0.09746730145453, 0.42279268784292,-0.02404522278963, 0.86845699519903, 0.07018010536676,-0.16312709919345, 0.15856593565846,-0.01648906828186
			,-0.41748239242452,-0.36422582893627, 0.32780006300773, 0.07051017250183,  0.6919803859936, 0.00465646030394, 0.04372937108135,-0.31599467827715
			,-0.30679576091205, 0.49791652481962, 0.03174244165381,-0.17591538889444, 0.00805755979924, 0.75337503399895, 0.21141292846411,-0.11690764004677
			,-0.45282163456062,-0.14149302373108,-0.77693758765435, -0.0840837448849,  0.1169123160509,-0.12369767073957, 0.28660242786034, 0.23047699494534
			,-0.41910366251916,-0.17337228543658, 0.50518695810173, 0.03189937328832, -0.3306359682238,-0.04824538675607, 0.26530527829483, 0.59668089171317
		};
		const double R[]{
			-1.52861288272238, -2.5747778739598,-2.46156397758862,-2.09447887349918,-2.98480523875039,-2.13186875924144
			,       0, 0.89201120339403, 0.46985861049624, 1.08208126371333,  1.3424295032531, 0.51428447049288
			,       0,       0, 0.15365213433261, 0.19193482979673, 0.27454106757704,-0.12378495493647
			,       0,       0,       0,       0,       0,       0
			,       0,       0,       0,       0,       0,       0
			,       0,       0,       0,       0,       0,       0
			,       0,       0,       0,       0,       0,       0
			,       0,       0,       0,       0,       0,       0 };
		const double U[]{
			-1.52861288272238,-2.57477787395980,-2.46156397758862,-2.09447887349918,-2.98480523875039,-2.13186875924144,
			0.27028485638057,0.89201120339403,0.46985861049624,1.08208126371333,1.34242950325310,0.51428447049288,
			0.11350675036463,-0.35372427934530,0.15365213433261,0.19193482979673,0.27454106757704,-0.12378495493647,
			0.06901373071131,-0.30391279940061,0.08007447911305,-0.00000000000000,0.00000000000000,0.00000000000000,
			0.29560700847904,0.33447158961584,-0.35116761638814,-0.00000000000000,-0.00000000000000,0.00000000000000,
			0.21723305878980,-0.33009195882314,0.04320017367392,-0.00000000000000,0.00000000000000,0.00000000000001,
			0.32062968689453,0.17204681507839,0.77144064562243,-0.00000000000000,0.00000000000000,0.00000000000000,
			0.29675498216921,0.19114160152386,-0.50321891935716,0.00000000000000,0.00000000000000,0.00000000000000 };
		const double tau[m]{
			-1.41228854678567,
			- 1.32938857711145,
			- 1.01012813942480,
			0.00000000000000,
			0.00000000000000,
			0.00000000000000,
			0.00000000000000,
			0.00000000000000 };
		const double U_p[m*n]{
			-3.28428867218801,-1.38922372415317,-2.13733332667495,-2.36182750504341,-2.70459558524877,-2.44199756429007,
			0.15441705339997,-0.63774194583297,-0.45405181155455,0.12459493100977,-0.27997260989751,-0.58063839076420,
			0.23794269315993,-0.25892822040419,-0.22431861580580,-0.02876684358782,-0.17872645347971,0.05639382214794,
			0.17362768639201,-0.22173062094000,0.12587511727245,-0.00000000000000,-0.00000000000000,0.00000000000000,
			0.17266197528614,0.31832433163261,-0.38718755972002,-0.00000000000000,-0.00000000000000,0.00000000000000,
			0.32464827088207,-0.20523286829108,0.09318674521862,-0.00000000000001,-0.00000000000000,0.00000000000000,
			0.19328559985062,0.33192834269179,0.66033201786009,0.00000000000000,-0.00000000000000,-0.00000000000000,
			0.23579295939196,0.18085907051168,-0.49923987562604,-0.00000000000000,-0.00000000000000,-0.00000000000000
		};
		const double tau_p[n]{
			-1.49390387270786,
			- 1.42597856157863,
			- 1.07542993033347,
			0.0,
			0.0,
			0.0,
		};
		const aris::Size p[n]{ 4,0,5,3,1,2 };
		const double b[m*rhs]{
			0.5060,0.1493,
			0.6991,0.2575,
			0.8909,0.8407,
			0.9593,0.2543,
			0.5472,0.8143,
			0.1386,0.2435,
			0.5472,0.8143,
			0.1386,0.2435
		};
		const double x_p[n*rhs]{
			-0.87181000800095, - 0.15999312309953,
			0.00000000000000,   0.00000000000000,
			0.00000000000000,   0.00000000000000,
			0.00000000000000,   0.00000000000000,
			- 0.08562266716248, - 0.28743665975638,
			1.34945506194506,   1.08231636915190,
		};
		const double x[n*rhs]{
			0.64629213948986,   1.46761105693367,
			1.30239264595087,   0.92123524540882,
			- 1.24013358689020, - 1.38551704040709,
			0.00000000000000,   0.00000000000000,
			0.00000000000000,  0.00000000000000,
			0.00000000000000,   0.00000000000000
		};
		const double solution_space[]{ -0.28667236101674,   -0.12825623053370, 1.08944890318972,
			0.00000000000000,   - 1.00000000000000,   0.00000000000000,
			0.00000000000000,   0.00000000000000, - 1.00000000000000,
			-1.00000000000000,   0.00000000000000, 0.00000000000000,
			0.75693278978173,   0.35923948569991,   0.44631766877724,
			0.12824099990312, 0.79675265843491,   - 0.25140054446824 };

		const double pinv[n*m]
		{
			-0.04709681913564,  0.247125799673245, -0.346877584025025, -0.352772995924887,  0.658647326647928, -0.317376796508276,  -0.24884071496572,  0.610002530571061
			,-0.0456275103415608,  0.133627982997883,  0.112345143400709,   0.22138335923286, -0.713178647219683,  0.159322279300896,   1.27623996673341, -0.936436098824208
			, 0.0587776043539234, 0.0762574744753922, -0.287565572844175, -0.393080270265156,   1.06339623987355, -0.293811635634574,  -1.20634932053306,   1.22835137951625
			,  0.122335993764335, -0.220126028877963,  0.249955287062167,  0.187971989797021, -0.105811160564627,  0.221726091712921, -0.406207315055435, 0.0529942518227027
			,  0.167571020045078, -0.251592832944579,  0.199672496084001, 0.0836654730517423,  0.263473639413296,   0.15970417003719, -0.969548900128279,  0.523516264862582
			, -0.140402541126088,  0.320934588340986,      -0.00486262925338471,  0.183346841061631, -0.907876680313713, 0.0768678750741675,   1.99889492179419,  -1.31316435069964
		};



		check_household(m, n, rhs, A, U, tau, Q, R, x, b, solution_space, pinv, U_p, tau_p, p, x_p, a_t, u_t, tau_t, q_t, r_t, x_t, b_t, ss_t, pinv_t);
	}

	check_household(0, 0, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, 1, 1, 1, 1, 1, 1, 1, 1, 1);
	
	Size p[1]{0};
	double ss[1]{ -1 };
	check_household(0, 1, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, ss, nullptr, nullptr, nullptr, p, nullptr, 1, 1, 1, 1, 1, 1, 1, 1, 1);
	
	
	double tau[1]{ 0 }, tau_p[1]{0};
	double q[1]{ 1 };
	check_household(1, 0, 0, nullptr, nullptr, tau, q, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, tau_p, nullptr, nullptr, 1, 1, 1, 1, 1, 1, 1, 1, 1);
}
void test_svd(){
	auto test_svd_mat = [](aris::Size m, aris::Size n, const double *A_old, auto a_t_old) {
		std::vector<double> A_data_((m + 1)*(n + 2)), U_data_((m + 3)*(m + 4)), S_data_((m + 5)*(n + 6)), V_data_((n + 7)*(n + 8));
		
		auto A = A_data_.data();
		auto U = U_data_.data();
		auto S = S_data_.data();
		auto V = V_data_.data();
		
		auto a_t = n;
		auto u_t = m;
		auto s_t = n;
		auto v_t = n;

		auto verify_result = [](aris::Size m, aris::Size n, const double *A_old, auto a_t_old, 
			double *A, auto a_t, double *U, auto u_t, double *S, auto s_t, double *V, auto v_t) 
		{
			s_mc(m, n, A_old, a_t_old, A, a_t);
			s_fill(m, m, -9999.0, U, u_t);
			s_fill(m, n, -9999.0, S, s_t);
			s_fill(n, n, -9999.0, V, v_t);

			s_svd(m, n, A, a_t, U, u_t, S, s_t, V, v_t);

			// verify result //
			std::vector<double> result1_data_(m*n), result2_data_(m*n), result3_data_(m*m), result4_data_(m * m), result5_data_(n * n), result6_data_(n * n);
			s_mm(m, n, m, U, u_t, S, s_t, result1_data_.data(), n);
			s_mm(m, n, n, result1_data_.data(), n, V, T(v_t), result2_data_.data(), n);
			if (!s_is_equal(m, n, result2_data_.data(), n, A, a_t, 1e-10)) {
				std::cout << "svd error 1!" << std::endl;
				dsp(m, n, result2_data_.data());
				dsp(m, n, A, a_t);
			}

			s_mm(m, m, m, U, u_t, U, T(u_t), result3_data_.data(), m);
			s_eye(m, result4_data_.data());
			if (!s_is_equal(m, m, result3_data_.data(), result4_data_.data(), 1e-10)) {
				std::cout << "svd error 2!" << std::endl;
				dsp(m, m, result3_data_.data());
			}

			s_mm(n, n, n, V, v_t, V, T(v_t), result5_data_.data(), n);
			s_eye(n, result6_data_.data());
			if (!s_is_equal(n, n, result5_data_.data(), result6_data_.data(), 1e-10)) {
				std::cout << "svd error 3!" << std::endl;
				dsp(n, n, result5_data_.data());
			}
		};

		verify_result(m, n, A_old, a_t_old, A, n, U, m, S, n, V, n);
		verify_result(m, n, A_old, a_t_old, A, n + 2, U, m + 4, S, n + 6, V, n + 8);
		verify_result(m, n, A_old, a_t_old, A, T(m + 1), U, T(m + 3), S, n + 6, V, n + 8);
		verify_result(m, n, A_old, a_t_old, A, T(m + 1), U, m + 4, S, T(m + 5), V, n + 8);
		verify_result(m, n, A_old, a_t_old, A, T(m + 1), U, m + 4, S, n + 6, V, T(n + 7));
		verify_result(m, n, A_old, a_t_old, A, T(m + 1), U, T(m + 3), S, T(m + 5), V, T(n + 7));
	};
	
	
	
	
	
	
	
	
	
	
	
	
	
	const aris::Size m{ 8 }, n{ 7 };
	
	// 共计 600 个数，最大支持25*24维
	double A[]{
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127,
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127, 
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127, 
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127, 
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127, 
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127, 
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127,
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,
		0.6280,0.4897,0.9427,0.1781,0.3689,0.4820,0.2653,0.8178,
		0.2920,0.3395,0.4177,0.1280,0.4607,0.1206,0.8244,0.2607,
		0.4317,0.9516,0.9831,0.9991,0.9816,0.5895,0.9827,0.5944,
		0.0155,0.9203,0.3015,0.1711,0.1564,0.2262,0.7302,0.0225,
		0.9841,0.0527,0.7011,0.0326,0.8555,0.3846,0.3439,0.4253,
		0.1672,0.7379,0.6663,0.5612,0.6448,0.5830,0.5841,0.3127, 
		0.5269,0.1062,0.2691,0.5391,0.8819,0.3763,0.2518,0.1078,
		0.4168,0.3724,0.4228,0.6981,0.6692,0.1909,0.2904,0.9063,
		0.6569,0.1981,0.5479,0.6665,0.1904,0.4283,0.6171,0.8797,};

	const double B[]{
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,};

	double C[25 * 24]{0.0};

	s_eye(24, C);

	
	//test_svd_mat(1, 1, A, 1);
	//test_svd_mat(2, 1, A, 1);
	//test_svd_mat(3, 1, A, 1);
	//test_svd_mat(5, 1, A, 1);
	//test_svd_mat(10, 1, A, 1);
	//test_svd_mat(1, 2, A, 2);
	//test_svd_mat(1, 3, A, 3);
	//test_svd_mat(1, 5, A, 5);
	//test_svd_mat(1, 10, A, 10);
	//test_svd_mat(2, 2, A, 2);
	//test_svd_mat(3, 2, A, 2);
	//test_svd_mat(4, 2, A, 2);
	//test_svd_mat(7, 2, A, 2);
	//test_svd_mat(10, 2, A, 2);
	//test_svd_mat(2, 3, A, 3);
	//test_svd_mat(2, 4, A, 4);
	//test_svd_mat(2, 7, A, 7);
	//test_svd_mat(2, 10, A, 10);
	//test_svd_mat(3, 3, A, 3);
	//test_svd_mat(4, 3, A, 3);
	//test_svd_mat(5, 3, A, 3);
	//test_svd_mat(7, 3, A, 3);
	//test_svd_mat(10, 3, A, 3);
	//test_svd_mat(3, 4, A, 4);
	//test_svd_mat(3, 5, A, 5);
	//test_svd_mat(3, 7, A, 7);
	//test_svd_mat(3, 10, A, 10);
	//test_svd_mat(5, 4, A, 4);
	//test_svd_mat(6, 5, A, 5);
	//test_svd_mat(7, 6, A, 6);
	//test_svd_mat(2, 1, A, 8);
	//test_svd_mat(3, 2, A, 8);
	//test_svd_mat(4, 3, A, 8);
	//test_svd_mat(8, 7, A, 8); 
	//test_svd_mat(10, 9, A, 9);
	//test_svd_mat(10, 10, A, 10);
	test_svd_mat(11, 10, A, 10);
	//test_svd_mat(10, 11, A, 11);
	//test_svd_mat(21, 10, A, 10);
	//test_svd_mat(10, 21, A, 21);
	//test_svd_mat(12, 11, A, 11);
	//test_svd_mat(13, 12, A, 12);
	//test_svd_mat(14, 13, A, 13);
	//test_svd_mat(15, 14, A, 14);
	//test_svd_mat(16, 15, A, 15);
	//test_svd_mat(17, 16, A, 16);
	test_svd_mat(18, 17, A, 17);
	//test_svd_mat(19, 18, A, 18);
	//test_svd_mat(20, 19, A, 19);
	//test_svd_mat(21, 20, A, 20);
	//test_svd_mat(22, 21, A, 21);
	//test_svd_mat(23, 22, A, 22);
	//test_svd_mat(24, 23, A, 23);
	//test_svd_mat(25, 24, A, 24);
	//test_svd_mat(24, 24, A, 24);
	//test_svd_mat(24, 25, A, 24);
	/*
	aris::dynamic::s_nm(25, 24, -1.0, A);
	test_svd_mat(1, 1, A, 1);
	test_svd_mat(2, 1, A, 1);
	test_svd_mat(3, 1, A, 1);
	test_svd_mat(5, 1, A, 1);
	test_svd_mat(10, 1, A, 1);
	test_svd_mat(1, 2, A, 2);
	test_svd_mat(1, 3, A, 3);
	test_svd_mat(1, 5, A, 5);
	test_svd_mat(1, 10, A, 10);
	test_svd_mat(2, 2, A, 2);
	test_svd_mat(3, 2, A, 2);
	test_svd_mat(4, 2, A, 2);
	test_svd_mat(7, 2, A, 2);
	test_svd_mat(10, 2, A, 2);
	test_svd_mat(2, 3, A, 3);
	test_svd_mat(2, 4, A, 4);
	test_svd_mat(2, 7, A, 7);
	test_svd_mat(2, 10, A, 10);
	test_svd_mat(3, 3, A, 3);
	test_svd_mat(4, 3, A, 3);
	test_svd_mat(5, 3, A, 3);
	test_svd_mat(7, 3, A, 3);
	test_svd_mat(10, 3, A, 3);
	test_svd_mat(3, 4, A, 4);
	test_svd_mat(3, 5, A, 5);
	test_svd_mat(3, 7, A, 7);
	test_svd_mat(3, 10, A, 10);
	test_svd_mat(5, 4, A, 4);
	test_svd_mat(6, 5, A, 5);
	test_svd_mat(7, 6, A, 6);
	test_svd_mat(2, 1, A, 8);
	test_svd_mat(3, 2, A, 8);
	test_svd_mat(4, 3, A, 8);
	test_svd_mat(8, 7, A, 8);
	test_svd_mat(10, 9, A, 9);
	test_svd_mat(10, 10, A, 10);
	test_svd_mat(11, 10, A, 10);
	test_svd_mat(10, 11, A, 11);
	test_svd_mat(21, 10, A, 10);
	test_svd_mat(10, 21, A, 21);
	test_svd_mat(12, 11, A, 11);
	test_svd_mat(13, 12, A, 12);
	test_svd_mat(14, 13, A, 13);
	test_svd_mat(15, 14, A, 14);
	test_svd_mat(16, 15, A, 15);
	test_svd_mat(17, 16, A, 16);
	test_svd_mat(18, 17, A, 17);
	test_svd_mat(19, 18, A, 18);
	test_svd_mat(20, 19, A, 19);
	test_svd_mat(21, 20, A, 20);
	test_svd_mat(22, 21, A, 21);
	test_svd_mat(23, 22, A, 22);
	test_svd_mat(24, 23, A, 23);
	test_svd_mat(25, 24, A, 24);
	test_svd_mat(24, 24, A, 24);
	test_svd_mat(24, 25, A, 24);

	test_svd_mat(2, 1, B, 1);
	test_svd_mat(3, 2, B, 2);
	test_svd_mat(4, 3, B, 3);
	test_svd_mat(5, 4, B, 4);
	test_svd_mat(6, 5, B, 5);
	test_svd_mat(7, 6, B, 6);
	test_svd_mat(2, 1, C, 24);
	test_svd_mat(5, 4, C + 24, 24);
	test_svd_mat(5, 4, C + 1, 24);
	test_svd_mat(4, 3, C + 24, 24);
	test_svd_mat(4, 3, C + 1, 24);
	test_svd_mat(4, 3, C, 3);
	test_svd_mat(5, 4, C, 4);
	test_svd_mat(6, 5, C, 5);
	test_svd_mat(7, 6, C, 6);
	test_svd_mat(4, 3, C, 24);
	test_svd_mat(2, 1, C, 8);
	test_svd_mat(3, 2, C, 8);
	test_svd_mat(4, 3, C, 8);
	test_svd_mat(8, 7, C, 8);
	test_svd_mat(10, 9, C, 9);
	test_svd_mat(11, 10, C, 10);
	test_svd_mat(12, 11, C, 11);
	test_svd_mat(13, 12, C, 12);
	test_svd_mat(14, 13, C, 13);
	test_svd_mat(15, 14, C, 14);
	test_svd_mat(16, 15, C, 15);
	test_svd_mat(17, 16, C, 16);
	test_svd_mat(18, 17, C, 17);
	test_svd_mat(19, 18, C, 18);
	test_svd_mat(20, 19, C, 19);
	test_svd_mat(21, 20, C, 20);
	test_svd_mat(22, 21, C, 21);
	test_svd_mat(23, 22, C, 22);
	test_svd_mat(24, 23, C, 23);
	test_svd_mat(25, 24, C, 24);

	*/
	

}
void test_interp_plane()
{
	const double x[]{   -2.7686,-2.9721,-3.0090,-3.0138,-2.9582,-2.9887,-2.9478,-2.9381,-2.1601,-1.8328
		,-1.8186,-1.8670,-1.7787,-1.9332,-2.0227,-2.1178,-0.7195,-0.8518,-0.8381,-1.0001
		,-1.2187,-1.2518,-0.9804,-0.9015, 0.1333, 0.0517, 0.0262, 0.1817, 0.1891,-0.2319
		,-0.0863, 0.1405, 1.0427, 1.2764, 0.7720, 0.8012, 1.2359, 1.0588, 0.9514, 0.7853
		, 2.2640, 1.8116, 2.1721, 2.2085, 2.0171, 1.9492, 2.2843, 1.7314, 3.0013, 3.1698
		, 3.2707, 3.1522, 2.8975, 2.7389, 3.0820, 3.1558, 4.0732, 4.2529, 3.8317, 4.2949
		, 4.2633, 3.9905, 4.0995, 3.7529, 4.9709, 4.7045, 5.1360, 4.8041, 5.2166, 4.7771
		, 4.8822, 5.2144};

	const double y[]{ -1.3368,-0.2600, 0.5999, 0.9295, 1.7047, 3.0369, 3.3140, 4.2657,-1.1911,-0.5924, 0.5877, 1.0795, 1.8048, 3.0439, 3.8939, 4.2536,-1.1992,-0.3481, 0.6500, 0.9759, 1.9498, 2.8249
		, 3.8241, 4.3243,-0.9403,-0.5023, 0.4518, 1.0726, 1.9673, 2.7137, 3.3429, 4.6630,-1.2817,-0.5154, 0.2329, 1.2027, 2.1011, 2.7503, 3.8938, 4.2506,-1.2510,-0.3468, 0.2325, 1.1333
		, 1.8810, 2.7834, 3.5903, 4.3095,-1.0504,-0.6438, 0.5204, 1.2440, 1.7190, 2.6600, 3.8072, 4.5987,-1.1631,-0.5441, 0.1589, 1.1461, 2.0320, 2.9280, 3.8806, 4.4921,-1.0902,-0.4447
		, 0.1326, 1.3247, 1.8256, 2.8339, 3.7759, 4.6096 };

	const double z[]{     -11.3658,-9.6420,-8.0893,-6.5908,-5.0941,-3.7174,-1.8666,-0.6087,-11.7614,-10.0730,-8.4697,-6.4883,-5.0814,-3.3811,-1.8939,-0.5904,-11.6232,-10.1793,-8.2949,-6.5381,-5.3437,-3.7126
		,-2.1569,-0.6385,-11.5949,-9.6553,-8.0452,-6.9484,-5.3841,-3.3908,-1.8392,-0.3418,-11.2490,-9.9588,-8.2480,-6.5307,-5.0071,-3.8593,-1.9401,-0.5302,-11.4103,-10.1777,-8.5371,-6.9694
		,-4.8964,-3.8734,-2.0803,-0.2940,-11.8123,-9.9179,-8.3191,-6.5184,-5.1578,-3.6344,-1.7932,-0.5630,-11.8498,-10.0409,-8.4346,-6.7693,-5.3293,-3.3248,-1.9662,-0.2669,-11.2924,-9.9514
		,-8.3858,-6.9809,-5.3106,-3.8028,-2.2244,-0.4296 };


	double p[5];
	s_interp_plane(72, x, y, z, p);

	//std::cout << s_interp_plane_error(72, x, y, z, p) << std::endl;

	double p0[3]{ 1.1,0.2,-3.5 };
	double p1[3]{ 2.1,0.34,-3.2 };
	double p2[3]{ 1.08,3.2,-3.4 };
	double p3[3]{ 1.12,0.21,-5.5 };

	auto ret = s_is_in_parallelepiped(p0, p1, p2, p3, std::array<double, 6>{0.0, 0.0, 0.0}.data());
	ret = s_is_in_parallelepiped(p0, p1, p2, p3, std::array<double, 6>{1.6, 1.2, -4.5}.data());
	ret = s_is_in_parallelepiped(p0, p1, p2, p3, std::array<double, 6>{5.6, 1.2, -4.5}.data());
	ret = s_is_in_parallelepiped(p0, p1, p2, p3, std::array<double, 6>{0.5, 1.2, -4.5}.data());
	ret = s_is_in_parallelepiped(p0, p1, p2, p3, std::array<double, 6>{1.6, -0.4, -4.5}.data());
	ret = s_is_in_parallelepiped(p0, p1, p2, p3, std::array<double, 6>{1.6, 3.6, -4.5}.data());
	ret = s_is_in_parallelepiped(p0, p1, p2, p3, std::array<double, 6>{1.6, 1.2, -6.5}.data());
	ret = s_is_in_parallelepiped(p0, p1, p2, p3, std::array<double, 6>{1.6, 1.2, -2.5}.data());

	ret = s_is_in_cylinder(p0, std::array<double, 3>{0.81,0,0}.data(), 0.3, 0.8, std::array<double, 6>{1.89, 0.2, -3.5}.data());
	ret = s_is_in_cylinder(p0, std::array<double, 3>{0.81, 0, 0}.data(), 0.3, 0.8, std::array<double, 6>{1.91, 0.2, -3.5}.data());

	ret = s_is_in_cylinder(p0, std::array<double, 3>{0.81, 0, 0}.data(), 0.3, 0.8, std::array<double, 6>{1.11, 0.25, -3.6}.data());
	ret = s_is_in_cylinder(p0, std::array<double, 3>{0.81, 0, 0}.data(), 0.3, 0.8, std::array<double, 6>{1.09, 0.2, -3.5}.data());

	ret = s_is_in_cylinder(p0, std::array<double, 3>{0.81, 0, 0}.data(), 0.3, 0.8, std::array<double, 6>{1.89, 0.49, -3.5}.data());
	ret = s_is_in_cylinder(p0, std::array<double, 3>{0.81, 0, 0}.data(), 0.3, 0.8, std::array<double, 6>{1.89, 0.51, -3.5}.data());

	ret = s_is_in_cylinder(p0, std::array<double, 3>{0.81, 0, 0}.data(), 0.3, 0.8, std::array<double, 6>{1.89, 0.2, -3.21}.data());
	ret = s_is_in_cylinder(p0, std::array<double, 3>{0.81, 0, 0}.data(), 0.3, 0.8, std::array<double, 6>{1.89, 0.2, -3.19}.data());


	const double pm[]{ 0.879923176281257, -0.0809848294377871 ,        0.468163071209206  ,       0.181716811988644,
		0.272192135295431      ,   0.893559408727084, -0.35701964169863,         0.619803060100149,
		-0.389418342308651     ,    0.441580163137156 ,        0.808307066774345, -0.763428560463371,
		0                      ,   0               ,          0       ,                  1 };


	const double plane1[]{ 0.181716811988644,         0.619803060100149 ,- 0.763428560463371,                       4.1 };

	double plane2[4];

	s_pm_dot_plane(pm, plane1, plane2);

	//dsp(1, 4, plane2);

}


void test_matrix()
{
	std::cout << std::endl << "-----------------test matrix--------------------" << std::endl;

	//test_basic_operation();
	//test_specific_matrix();
	//test_multiply();
	//test_llt();
	//test_householder();
	test_svd();
	//test_interp_plane();

	std::cout << "-----------------test matrix finished-----------" << std::endl << std::endl;
}