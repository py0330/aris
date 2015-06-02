#ifndef ARIS_PLAN_H_
#define ARIS_PLAN_H_

#include"Aris_DynKer.h"

#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

namespace Aris
{
	namespace Plan
	{
		template<unsigned int inDim, unsigned int eeDim>
		class TRAJECTORY_GENERATOR
		{
		public:
			double inVelMax[inDim], inVelMin[inDim], inAccMax[inDim], inAccMin[inDim];
			double dt;
			double beginS, beginDotS, endS, endDotS;
			double maxDotS;
			int realTotalNum, totalNum;

		private:
			double jacInv[inDim][eeDim], dJacInv[inDim][eeDim], cV[inDim], cA[inDim], g[eeDim], h[eeDim];
			double eePos[eeDim], eeVel[eeDim], inPos[inDim], inVel[inDim];			
			
			enum
			{
				MAX_PLAN_NUM = 3000,
				MAX_PLAN_INTERVAL_NUM = 10
			};

		public:
			double s[MAX_PLAN_NUM * 2], ds[MAX_PLAN_NUM * 2], dds[MAX_PLAN_NUM * 2];
			double final_s[MAX_PLAN_NUM * 2], final_ds[MAX_PLAN_NUM * 2], final_dds[MAX_PLAN_NUM * 2];

			double maxDs[MAX_PLAN_NUM * 2];
		private:

			
			int interval[MAX_PLAN_INTERVAL_NUM + 1];
			double intervalS[MAX_PLAN_INTERVAL_NUM + 1];
			double intervalDotS[MAX_PLAN_INTERVAL_NUM + 1];
			int intervalNum;


		private:
			double s_itv[MAX_PLAN_INTERVAL_NUM][2*MAX_PLAN_NUM];
			double ds_itv[MAX_PLAN_INTERVAL_NUM][2*MAX_PLAN_NUM];
			double dds_itv[MAX_PLAN_INTERVAL_NUM][2*MAX_PLAN_NUM];

			int f_itv[MAX_PLAN_INTERVAL_NUM];
			int b_itv[MAX_PLAN_INTERVAL_NUM];
			bool f_finished[MAX_PLAN_INTERVAL_NUM];
			bool b_finished[MAX_PLAN_INTERVAL_NUM];
			
		public:
			int(*GetEveryThing)(double t_in, double s_in, double dotS_in
				, double* jacInv_out, double* cV_out, double* cA_out
				, double* g_out, double* h_out);

		public:
			int ComputeNextPnt(bool min_f_or_b, int index, double &maxL, double &minR)
			{
				if (min_f_or_b)
				{
					ComputeBund(0, s_itv[index][f_itv[index]], ds_itv[index][f_itv[index]], maxL, minR);
					CheckBundWithMaxDs(min_f_or_b, index, maxL, minR);

					dds_itv[index][f_itv[index]] = minR;
					
					ds_itv[index][f_itv[index] + 1] = ds_itv[index][f_itv[index]] + dt*dds_itv[index][f_itv[index]];
					s_itv[index][f_itv[index] + 1] = s_itv[index][f_itv[index]] + dt*ds_itv[index][f_itv[index]] + 0.5*dt*dt*ds_itv[index][f_itv[index]];

					f_itv[index]++;
				}
				else
				{
					ComputeBund(0, s_itv[index][b_itv[index]], ds_itv[index][b_itv[index]], maxL, minR);
					CheckBundWithMaxDs(min_f_or_b, index, maxL, minR);

					dds_itv[index][b_itv[index]] = maxL;

					ds_itv[index][b_itv[index] - 1] = ds_itv[index][b_itv[index]] - dt*dds_itv[index][b_itv[index]];
					s_itv[index][b_itv[index] - 1] = s_itv[index][b_itv[index]] - dt*ds_itv[index][b_itv[index]] + 0.5*dt*dt*ds_itv[index][b_itv[index]];

					b_itv[index]--;
				}

				return 0;
			};

			int ComputeBund(double time_in, double s_in, double ds_in, double &maxL_out, double &minR_out)
			{
				double l1[inDim], l2[inDim], r1[inDim], r2[inDim];
				double J_dot_g[inDim], J_dot_h[inDim];
				
				GetEveryThing(time_in, s_in, ds_in, *jacInv, cV, cA, g, h);

				Aris::DynKer::s_dgemm(inDim, 1, eeDim, 1, *jacInv, eeDim, g, 1, 0, J_dot_g, 1);
				Aris::DynKer::s_dgemm(inDim, 1, eeDim, 1, *jacInv, eeDim, h, 1, 0, J_dot_h, 1);

				for (int i = 0; i < inDim; ++i)
				{
					if (J_dot_g[i]<0)
					{
						l1[i] = (inAccMax[i] - cA[i] - J_dot_h[i] * ds_in * ds_in) / J_dot_g[i];
						r1[i] = (inAccMin[i] - cA[i] - J_dot_h[i] * ds_in * ds_in) / J_dot_g[i];
						l2[i] = (inVelMax[i] - cV[i] - J_dot_g[i] * ds_in) / J_dot_g[i] / dt;
						r2[i] = (inVelMin[i] - cV[i] - J_dot_g[i] * ds_in) / J_dot_g[i] / dt;
					}
					else
					{
						l1[i] = (inAccMin[i] - cA[i] - J_dot_h[i] * ds_in * ds_in) / J_dot_g[i];
						r1[i] = (inAccMax[i] - cA[i] - J_dot_h[i] * ds_in * ds_in) / J_dot_g[i];
						l2[i] = (inVelMin[i] - cV[i] - J_dot_g[i] * ds_in) / J_dot_g[i] / dt;
						r2[i] = (inVelMax[i] - cV[i] - J_dot_g[i] * ds_in) / J_dot_g[i] / dt;
					}
				}

				maxL_out = l1[0];
				minR_out = r1[0];

				for (int i = 0; i < inDim; ++i)
				{
					if (l1[i]>maxL_out)maxL_out = l1[i];
					if (l2[i]>maxL_out)maxL_out = l2[i];
					if (r1[i]<minR_out)minR_out = r1[i];
					if (r2[i]<minR_out)minR_out = r2[i];
				}

				return 0;

			}

			int CheckBundWithMaxDs(bool min_f_or_b,int index, double &maxL_out, double &minR_out)
			{
				double s_in, ds_in;
				if (min_f_or_b)
				{
					s_in = s_itv[index][f_itv[index]];
					ds_in = ds_itv[index][f_itv[index]];
				}
				else
				{
					s_in = s_itv[index][b_itv[index]];
					ds_in = ds_itv[index][b_itv[index]];
				}
				
				
				/*检查在这些加速度中，是否可能会让隔壁的速度超限*/
				double nextS, lastS;
				double realID;
				int fid,eid;
				double fFactor, eFactor;
				double maxRealDs;
				double maxRealDDs;

				nextS = s_in + ds_in*dt;
				lastS = s_in - ds_in*dt;


				/*先计算下一个点*/
				realID = (nextS - beginS) / (endS - beginS)*totalNum - 1;
				fid = (int)realID;
				eid = fid + 1;

				if (fid < 0)
				{
					fid = 0;
					eid = 0;
				}

				if (eid>totalNum - 1)
				{
					fid = totalNum - 1;
					eid = totalNum - 1;
				}

				fFactor = 1 - (realID - fid);
				eFactor = 1 - fFactor;

				maxRealDs = fFactor*maxDs[fid] + eFactor*maxDs[eid];

				maxRealDDs = (maxRealDs - ds_in) / dt;
				if (maxRealDDs < minR_out)
					minR_out = maxRealDDs;



				/*再计算上一个点*/
				realID = (lastS - beginS) / (endS - beginS)*totalNum - 1;
				fid = (int)realID;
				eid = fid + 1;

				if (fid < 0)
				{
					fid = 0;
					eid = 0;
				}

				if (eid>totalNum - 1)
				{
					fid = totalNum - 1;
					eid = totalNum - 1;
				}

				fFactor = 1 - (realID - fid);
				eFactor = 1 - fFactor;

				maxRealDs = fFactor*maxDs[fid] + eFactor*maxDs[eid];

				maxRealDDs = (ds_in-maxRealDs) / dt;
				if (maxRealDDs > maxL_out)
					maxL_out = maxRealDDs;


				return 0;
			}

			int FindInterval()
			{
				double loc_s, loc_ds;
				double maxL, minR;
				double lowerBund, higherBund;

				double precision = 0.00001;

				/*对maxDs赋初值*/
				for (int i = 0; i < totalNum; ++i)
				{
					maxDs[i] = maxDotS;
				}
				

				bool ifNeedIterative = true;
				do
				{
					ifNeedIterative = false;

					for (int i = 0; i < totalNum; ++i)
					{
						loc_s = beginS + (endS - beginS)*(i + 1) / totalNum;

						lowerBund = 0;
						higherBund = maxDs[i];
						loc_ds = higherBund;
						while (abs(lowerBund - higherBund)>precision)
						{
							ComputeBund(0, loc_s, loc_ds, maxL, minR);

							bool ifOK;

							if (maxL > minR)
							{
								ifOK = false;
							}
							else if (i == 0)
							{
								if (loc_ds + maxL*dt>maxDs[i + 1])
								{
									ifOK = false;
								}
								else
								{
									ifOK = true;
								}
							}
							else if (i == totalNum - 1)
							{
								if (loc_ds - minR*dt > maxDs[i - 1])
								{
									ifOK = false;
								}
								else
								{
									ifOK = true;
								}
							}
							else
							{
								if ((loc_ds - minR*dt>maxDs[i - 1]) || (loc_ds + maxL*dt>maxDs[i + 1]))
								{
									ifOK = false;
								}
								else
								{
									ifOK = true;
								}
							}

							if (ifOK)
							{
								lowerBund = loc_ds;
							}
							else
							{
								higherBund = loc_ds;
								ifNeedIterative = true;
							}

							loc_ds = (lowerBund + higherBund) / 2.0;
						}
						maxDs[i] = loc_ds;
					}
					for (int i = totalNum-1; i > -1; --i)
					{
						loc_s = beginS + (endS - beginS)*(i + 1) / totalNum;

						lowerBund = 0;
						higherBund = maxDs[i];
						loc_ds = higherBund;
						while (abs(lowerBund - higherBund)>precision)
						{
							ComputeBund(0, loc_s, loc_ds, maxL, minR);

							bool ifOK;

							if (maxL > minR)
							{
								ifOK = false;
							}
							else if (i == 0)
							{
								if (loc_ds + maxL*dt>maxDs[i + 1])
								{
									ifOK = false;
								}
								else
								{
									ifOK = true;
								}
							}
							else if (i == totalNum - 1)
							{
								if (loc_ds - minR*dt > maxDs[i - 1])
								{
									ifOK = false;
								}
								else
								{
									ifOK = true;
								}
							}
							else
							{
								if ((loc_ds - minR*dt>maxDs[i - 1]) || (loc_ds + maxL*dt>maxDs[i + 1]))
								{
									ifOK = false;
								}
								else
								{
									ifOK = true;
								}
							}

							if (ifOK)
							{
								lowerBund = loc_ds;
							}
							else
							{
								higherBund = loc_ds;
								ifNeedIterative = true;
							}

							loc_ds = (lowerBund + higherBund) / 2.0;
						}
						maxDs[i] = loc_ds;
					}
					int jjjj=0;
					jjjj++;
					//memcpy(maxDs, maxDs, sizeof(double)*totalNum);
				} while (ifNeedIterative);

				/*判断是否为极小值点*/
				intervalNum = 0;
				interval[intervalNum] = 0;
				intervalS[intervalNum] = beginS;
				intervalDotS[intervalNum] = beginDotS;
				intervalNum++;

				for (int i = 1; i < totalNum-1; ++i)
				{
					if ((maxDs[i] <= maxDs[i-1]) && (maxDs[i] < maxDs[i+1]))
					{
						interval[intervalNum] = i;
						intervalS[intervalNum] = beginS + (endS - beginS)*(i + 1) / totalNum;
						intervalDotS[intervalNum] = maxDs[i];
						intervalNum++;
					}
				}

				interval[intervalNum] = totalNum - 1;
				intervalS[intervalNum] = endS;
				intervalDotS[intervalNum] = endDotS;
				intervalNum++;

				return 0;
			}

			int Run()
			{
				/*初始化*/
				FindInterval();

				for (int i = 0; i < intervalNum; ++i)
				{
					f_itv[i] = MAX_PLAN_NUM;
					b_itv[i] = MAX_PLAN_NUM;
					s_itv[i][MAX_PLAN_NUM] = intervalS[i];
					ds_itv[i][MAX_PLAN_NUM] = intervalDotS[i];
					f_finished[i] = false;
					b_finished[i] = false;
				}

				f_finished[intervalNum - 1] = true;
				b_finished[0] = true;

				bool finished = false;
				double minCurrentDs;
				bool min_f_or_b;
				int min_index;

				int testNum = 0;
				/*开始主循环迭代搜寻轨迹*/
				while (!finished)
				{
					/*寻找最小迭代点*/
					minCurrentDs = maxDotS;
					for (int i = 0; i < intervalNum; ++i)
					{
						if ((ds_itv[i][f_itv[i]] < minCurrentDs) && (f_finished[i] == false))
						{
							min_f_or_b = true;
							min_index = i;
							minCurrentDs = ds_itv[i][f_itv[i]];
						}

						if ((ds_itv[i][b_itv[i]] < minCurrentDs) && (b_finished[i] == false))
						{
							min_f_or_b = false;
							min_index = i;
							minCurrentDs = ds_itv[i][b_itv[i]];
						}
					}

					/*开始对迭代点进行迭代*/
					double maxL, minR;
					ComputeNextPnt(min_f_or_b, min_index, maxL, minR);


					/*开始判断是否两条曲线已经接近*/

					if (min_f_or_b)
					{
						/*判断位置是否接近*/
						if (s_itv[min_index][f_itv[min_index]] > s_itv[min_index + 1][b_itv[min_index + 1]])
						{
							/*判断速度是否接近*/
							if (ds_itv[min_index][f_itv[min_index]] > ds_itv[min_index + 1][b_itv[min_index + 1]])
							{
								/*如果接近，则说明相邻两个区间已经连接好*/
								f_finished[min_index] = true;
								b_finished[min_index + 1] = true;
							}
							else
							{
								/*如果不接近，说明双方速度不符*/
								if (b_itv[min_index + 1] < MAX_PLAN_NUM)
								{
									/*若对方已经向自己前进，则逼对方后退*/
									b_itv[min_index + 1]++;
								}
								else
								{
									/*如果对方退无可退，则改变对方初值*/
									if (min_index < (intervalNum - 2))
									{
										f_finished[min_index + 1] = false;
										b_finished[min_index + 2] = false;
									}

									f_itv[min_index + 1] = MAX_PLAN_NUM;
									b_itv[min_index + 1] = MAX_PLAN_NUM;

									ds_itv[min_index + 1][MAX_PLAN_NUM] = ds_itv[min_index][f_itv[min_index]];
								}
							}
						}
					}
					else
					{
						if (s_itv[min_index][b_itv[min_index]] < s_itv[min_index - 1][f_itv[min_index - 1]])
						{
							/*判断速度是否接近*/
							if (ds_itv[min_index][b_itv[min_index]] > ds_itv[min_index - 1][f_itv[min_index - 1]])
							{
								b_finished[min_index] = true;
								f_finished[min_index - 1] = true;
							}
							else
							{
								/*如果不接近，说明双方速度不符*/
								if (f_itv[min_index - 1] > MAX_PLAN_NUM)
								{
									/*若对方已经向自己前进，则逼对方后退*/
									f_itv[min_index - 1]--;
								}
								else
								{
									/*如果对方退无可退，则改变对方初值*/
									if (min_index > 1)
									{
										b_finished[min_index - 1] = false;
										f_finished[min_index - 2] = false;
									}

									f_itv[min_index - 1] = MAX_PLAN_NUM;
									b_itv[min_index - 1] = MAX_PLAN_NUM;

									ds_itv[min_index - 1][MAX_PLAN_NUM] = ds_itv[min_index][b_itv[min_index]];
								}
							}
						}
					}

					/*判断是否全部结束*/
					finished = true;
					for (int i = 0; i < intervalNum; ++i)
					{
						if (f_finished[i] == false)
						{
							finished = false;
							break;
						}

						if (b_finished[i] == false)
						{
							finished = false;
							break;
						}
					}


					if ((testNum >0)
						&& (testNum % 100 == 0))
					{
						cout << testNum << endl;
					}
					testNum++;
				}


				/*主循环结束，以下将各段轨迹拼接起来*/


				/*首先将后续所有轨迹添加偏置，使得位置连续*/
				for (int i = 1; i < intervalNum; ++i)
				{
					/*判断前一个轨迹的速度高还是后一个轨迹的速度高，用低的速度当为连接处的速度*/
					double diff;

					if (ds_itv[i - 1][f_itv[i - 1]]>ds_itv[i][b_itv[i]])
					{
						diff = s_itv[i - 1][f_itv[i - 1] - 1]
							+ 0.5*(ds_itv[i][b_itv[i]] + ds_itv[i - 1][f_itv[i - 1] - 1]) * dt
							- s_itv[i][b_itv[i]];
					}
					else
					{
						diff = s_itv[i - 1][f_itv[i - 1]]
							+ 0.5*(ds_itv[i - 1][f_itv[i - 1]] + ds_itv[i][b_itv[i] + 1]) * dt
							- s_itv[i][b_itv[i] + 1];
					}

					for (int j = b_itv[i]; j < f_itv[i] + 1; ++j)
					{
						s_itv[i][j] += diff;
					}
				}

				/*将所有连续的数据拷贝到同一内存中，这里舍弃下一段轨迹和当前轨迹中速度较高的那个点*/
				/*首先拷入第一段轨迹*/
				int index = 0;
				memcpy(&s[index], &s_itv[0][b_itv[0]], sizeof(double)*(f_itv[0] - b_itv[0] + 1));
				memcpy(&ds[index], &ds_itv[0][b_itv[0]], sizeof(double)*(f_itv[0] - b_itv[0] + 1));
				index += f_itv[0] - b_itv[0] + 1;

				for (int i = 1; i < intervalNum; ++i)
				{
					/*若当前轨迹中速度大于下一个点的速度，那么去掉当前点中的一个点*/
					if (ds[index - 1]>ds_itv[i][b_itv[i]])
					{
						memcpy(&s[index - 1], &s_itv[i][b_itv[i]], sizeof(double)*(f_itv[i] - b_itv[i] + 1));
						memcpy(&ds[index - 1], &ds_itv[i][b_itv[i]], sizeof(double)*(f_itv[i] - b_itv[i] + 1));
						index += f_itv[i] - b_itv[i];
					}
					else
					{
						memcpy(&s[index], &s_itv[i][b_itv[i] + 1], sizeof(double)*(f_itv[i] - b_itv[i]));
						memcpy(&ds[index], &ds_itv[i][b_itv[i] + 1], sizeof(double)*(f_itv[i] - b_itv[i]));
						index += f_itv[i] - b_itv[i];
					}
				}

				realTotalNum = index;

				/*其次将轨迹数目变为所需要的数目，轨迹长度对应好末端数值*/
				double valueScale = (endS - beginS) / (s_itv[intervalNum - 1][f_itv[intervalNum - 1]] - beginS);
				double timeScale = ((double)totalNum) / ((double)realTotalNum - 1);

				for (int i = 0; i < totalNum; ++i)
				{
					int fid = (int)((i + 1) / timeScale);
					double factor1 = ((i + 1) / timeScale - fid);
					double factor2 = (1 - (i + 1) / timeScale + fid);

					if (fid >= realTotalNum - 1)
					{
						fid = realTotalNum - 1;
						factor1 = 0;
						factor2 = 1;
						s[fid + 1] = 0;
					}

					if (fid >= realTotalNum - 2)
					{
						int j = 0;
						j++;

					}

					final_s[i] = ((s[fid + 1] * factor1 + s[fid] * factor2) - beginS)*valueScale + beginS;
				}

				/*补偿dotS，使得轨迹开始和结束的导数为要求数值*/
				double bScale = (1 - 1 / timeScale * valueScale)*beginDotS;
				double eScale = (1 - 1 / timeScale * valueScale)*endDotS;

				double b = -1;
				double e = totalNum - 1;


				for (int i = 0; i < totalNum; ++i)
				{
					/*这里用sin((s-e)^2*pi/(e-b)^2)/pi/2和sin((s-b)^2*pi/(e-b)^2)/pi/2来补偿
					这个函数的好处是不改变初值的情况下分别改变前和后的导数*/
					double real = sin((i - e)*(i - e) * PI / (e - b) / (e - b)) / PI / 2 * (totalNum - 1)*dt;

					final_s[i] += bScale*sin((i - e)*(i - e) * PI / (e - b) / (e - b)) / PI / 2 * (totalNum - 1)*dt;
					final_s[i] -= eScale*sin((i - b)*(i - b) * PI / (e - b) / (e - b)) / PI / 2 * (totalNum - 1)*dt;
				}

				/*检查迭代过程中开始点和结束点的导数是否被改过*/
				if ((ds_itv[0][b_itv[0]] == beginDotS) && (ds_itv[intervalNum - 1][f_itv[intervalNum - 1]] == endDotS))
				{
					cout << "Trace is good" << endl;
				}
				else
				{
					if (ds_itv[0][b_itv[0]] != beginDotS)
						cout << "Begin trace is bad" << endl;
					if (ds_itv[intervalNum - 1][f_itv[intervalNum - 1]] != endDotS)
						cout << "End trace is bad" << endl;
				}


				return 0;
			}

			int RunInMovingMarker()
			{
				/*使用lambda写的嵌套函数*/
				auto ComputeNextPntInMovingMarker = [this](bool min_f_or_b, int index, double dds)
				{
					if (min_f_or_b)
					{
						this->dds[index] = dds;
						this->ds[index + 1] = this->ds[index] + dds*dt;
						this->s[index + 1] = this->s[index] + 0.5*(this->ds[index] + this->ds[index + 1])*this->dt;
					}
					else
					{
						this->dds[index] = dds;
						this->ds[index - 1] = this->ds[index] - dds*dt;
						this->s[index - 1] = this->s[index] - 0.5*(this->ds[index] + this->ds[index - 1])*this->dt;
					}

					return 0;
				};

				/*Range用来记录前段中加速和后段中减速的队列*/
				struct Range
				{
					int beginId,length;
				};
				
				/*初始化*/
				std::vector<Range> accRanges,decRanges;
				Range range = { 0, 0 };
				accRanges.push_back(range);
				range.beginId = totalNum - 1;
				decRanges.push_back(range);

				
				s[0] = beginS;
				ds[0] = beginDotS;

				s[totalNum - 1] = endS;
				ds[totalNum - 1] = endDotS;

				int fid = 0;
				int eid = totalNum - 1;

				/*主迭代循环*/
				while (true)
				{
					if ((eid-fid)%100==0)
					{
						int jjjj = 0;
						jjjj++;
					}

					if ((eid - fid) % 10 == 0)
					{
						int jjjj = 0;
						jjjj++;
					}

					double maxL, minR;
					
					if (ds[fid] <= ds[eid])
					{
						ComputeBund((fid + 1)*dt, s[fid], ds[fid], maxL, minR);
						while (maxL>minR)
						{
							Range *r = &accRanges.back();
							
							if (accRanges.back().length>0)
							{
								accRanges.back().length--;
							}
							else
							{
								accRanges.pop_back();
								accRanges.back().length--;
							}

							if (accRanges.back().length == 0)
								accRanges.pop_back();


							for (int i = accRanges.back().beginId + accRanges.back().length; i < fid; ++i)
							{
								ComputeBund((i + 1)*dt, s[i], ds[i], maxL, minR);
								double dds_ = maxL;
								ComputeNextPntInMovingMarker(true, i, dds_);
							}

							ComputeBund((fid + 1)*dt, s[fid], ds[fid], maxL, minR);
						}

						ComputeNextPntInMovingMarker(true, fid, minR);

						if (accRanges.back().beginId + accRanges.back().length == fid)
						{
							accRanges.back().length++;
						}
						else
						{
							range.beginId = fid;
							range.length = 1;
							accRanges.push_back(range);
						}

						fid++;
					}
					else
					{
						ComputeBund((eid + 1)*dt, s[eid], ds[eid], maxL, minR);
						while (maxL>minR)
						{
							if (decRanges.back().length>0)
							{
								decRanges.back().length--;
							}
							else
							{
								decRanges.pop_back();
								decRanges.back().length--;
							}

							if (decRanges.back().length == 0)
								decRanges.pop_back();


							for (int i = decRanges.back().beginId - decRanges.back().length; i > eid; --i)
							{
								ComputeBund((i + 1)*dt, s[i], ds[i], maxL, minR);
								double dds_ = minR;
								ComputeNextPntInMovingMarker(false, i, dds_);
							}

							ComputeBund((eid + 1)*dt, s[eid], ds[eid], maxL, minR);
						}

						ComputeNextPntInMovingMarker(false, eid, maxL);

						if (decRanges.back().beginId - decRanges.back().length == eid)
						{
							decRanges.back().length++;
						}
						else
						{
							range.beginId = eid;
							range.length = 1;
							decRanges.push_back(range);
						}

						eid--;
					}

					

					/*判断是否位置接近*/
					if (s[fid] > s[eid]) 
					{
						/*判断是否速度接近*/
						if (ds[fid] > ds[eid])
						{
							ComputeBund((eid + 1)*dt, s[eid], ds[eid], maxL, minR);
							if (((ds[eid] - ds[fid])<minR) && ((ds[eid] - ds[fid])>maxL))
							{
								break;
							}
							else
							{
								fid--;
							}
						}
						else
						{
							ComputeBund((fid + 1)*dt, s[fid], ds[fid], maxL, minR);
							if (((ds[eid] - ds[fid])<minR*dt) && ((ds[eid] - ds[fid])>maxL*dt))
							{
								break;
							}
							else
							{
								eid++;
							}
						}
					}
				}

				/*首先将后半段所有轨迹添加偏置，使得位置连续，之后连接两段轨迹*/
				double diff;

				if (ds[fid]>ds[eid])
				{
					fid--;
				}
				else
				{
					eid++;
				}

				diff = s[fid] + 0.5*(ds[fid] + ds[eid]) * dt - s[eid];

				for (int j = eid; j < totalNum; ++j)
				{
					s[j] += diff;
				}

				memcpy(&s[fid + 1], &s[eid], (totalNum - eid)*sizeof(double));



				/*连接两段轨迹*/
				/*由于起始点应从轨迹中去除，因此实际上规划出来的数目应为totalNum-1*/
				realTotalNum = totalNum - eid + fid;

				double valueScale = (endS - beginS) / (s[realTotalNum - 1] - beginS);
				double timeScale = ((double)totalNum) / (realTotalNum);

				for (int i = 0; i < totalNum; ++i)
				{
					int dataId = (int)((i + 1) / timeScale);
					double factor1 = ((i + 1) / timeScale - dataId);
					double factor2 = (1 - (i + 1) / timeScale + dataId);

					if (dataId >= realTotalNum)
					{
						dataId = realTotalNum;
						factor1 = 0;
						factor2 = 1;
						s[dataId + 1] = 0;
					}

					if (dataId >= realTotalNum - 2)
					{
						int j = 0;
						j++;

					}

					final_s[i] = ((s[dataId + 1] * factor1 + s[dataId] * factor2) - beginS)*valueScale + beginS;
				}

				return 0;
			}
		};

		double acc_up(unsigned int n, unsigned int i);
		double acc_down(unsigned int n, unsigned int i);
		double dec_up(unsigned int n, unsigned int i);
		double dec_down(unsigned int n, unsigned int i);

		double acc_even(unsigned int n, unsigned int i);
		double dec_even(unsigned int n, unsigned int i);
		double even(unsigned int n, unsigned int i);
	}
}


#endif