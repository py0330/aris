#define linux 1

#include "aris_sensor_vision.h"
#include <vector>
#include <string>
#include <algorithm>
#include <XnCppWrapper.h>
#include "math.h"




using namespace xn;

namespace aris
{

	namespace sensor
	{

		struct Point3D
		{
			double X;
			double Y;
			double Z;
		};


		void MatrixMultiple(double A[4][4], double B[4][4], double C[4][4])
		{
			for(int i = 0; i < 4; i++)
			{
				for(int j = 0; j < 4; j++)
				{
					for(int k = 0; k < 4; k++)
					{
						C[i][j] += A[i][k] * B[k][j];
					}
				}
			}
		}

		void GeneratePointCloud(DepthGenerator& rDepthGen, const XnDepthPixel* pDepth, vector<Point3D>& vPointCloud)
		{
			DepthMetaData mDepthMD;
			rDepthGen.GetMetaData(mDepthMD);
			unsigned int uPointNum = mDepthMD.FullXRes() * mDepthMD.FullYRes();

			XnPoint3D* pDepthPointSet = new XnPoint3D[uPointNum];
			unsigned int i, j, idxshift, idx;
			for( j = 0; j < mDepthMD.FullYRes(); ++j)
			{
				idxshift = j * mDepthMD.FullXRes();

				for(i = 0; i < mDepthMD.FullXRes(); ++i)
				{
					idx = idxshift + i;
					pDepthPointSet[idx].X = i;
					pDepthPointSet[idx].Y = j;
					pDepthPointSet[idx].Z = pDepth[idx];
				}
			}

			XnPoint3D* p3DPointSet = new XnPoint3D[uPointNum];

			rDepthGen.ConvertProjectiveToRealWorld(uPointNum, pDepthPointSet, p3DPointSet);

			delete[] pDepthPointSet;

			for(i = 0; i < uPointNum; ++i)
			{
				if(p3DPointSet[i].Z == 0)
					continue;

				Point3D tempPoint;
				tempPoint.X = p3DPointSet[i].X;
				tempPoint.Y = p3DPointSet[i].Y;
				tempPoint.Z = p3DPointSet[i].Z;

				vPointCloud.push_back(tempPoint);
			}

			delete[] p3DPointSet;
		}

		void GenerateGridMap(VISION_DATA &cdata, vector<Point3D>& vPointClound)
		{
			int cGridNum[120][120] = {0};

			for(int i = 0; i < vPointClound.size(); ++i)
			{
				if(vPointClound[i].X>-1.5&&vPointClound[i].X<1.5&&
						vPointClound[i].Z>0&&vPointClound[i].Z<3)
				{
					int m, n;
					n = floor(vPointClound[i].X/0.025) + 60;
					m = floor(vPointClound[i].Z/0.025);

					//Mean

					cdata.gridMap[m][n] = (cdata.gridMap[m][n]*cGridNum[m][n] + vPointClound[i].Y)/(cGridNum[m][n] + 1);
					cGridNum[m][n] = cGridNum[m][n] + 1;

					//Max

					//if (GridMap(m,n) < RobotPoint->points[i].y)
					//{
					//GridMap(m,n) = RobotPoint->points[i].y;
					//}
				}
			}
		}

		class KINECT::KINECT_STRUCT
		{
			friend class KINECT;

		public:
			XnStatus mStatus;
			Context mContext;
			DepthGenerator mDepthGenerator;
			XnMapOutputMode mapDepthMode;
			void CheckOpenNIError(XnStatus eResult, string sStatus);
		private:
			vector<Point3D> v1PointCloud;
			vector<Point3D> v2PointCloud;
		};

		void KINECT::KINECT_STRUCT::CheckOpenNIError(XnStatus eResult, string sStatus)
		{
			if(eResult != XN_STATUS_OK)
			{
				cerr << sStatus << "  Error" << xnGetStatusString(eResult) << endl;
			}
			else
			{
				cout<< sStatus << " Successful " << xnGetStatusString(eResult) << endl;
			}
		}

		KINECT::KINECT():mKinectStruct(new KINECT_STRUCT)
		{
			mKinectStruct->mStatus = XN_STATUS_OK;

			double  locKinectToRobot[4][4] = { { 0.9995, 0.0134, -0.0273, 0.0224 },
			{ -0.0304, 0.5120, -0.8584, 0.2026 + 0.038 },
			{ 0.0025, 0.8589, 0.5122, 0.5733 },
			{ 0, 0, 0, 1 } };

			std::copy_n(static_cast<double *>(*locKinectToRobot), 16, static_cast<double *>(*this->kinectToRobot));

			

		}

		void KINECT::init()
		{
			mKinectStruct->mStatus = mKinectStruct->mContext.Init();
			mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "initialize context");
			mKinectStruct->mapDepthMode.nFPS = 30;
			mKinectStruct->mapDepthMode.nXRes = 640;
			mKinectStruct->mapDepthMode.nYRes = 480;

			mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.Create(mKinectStruct->mContext);
			mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Create depth Generator");
			mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.SetMapOutputMode(mKinectStruct->mapDepthMode);
			mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Map Mode Set");
			mKinectStruct->mStatus  = mKinectStruct->mContext.StartGeneratingAll();
			mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Start View Cloud");
		}

		void KINECT::release()
		{
			mKinectStruct->mContext.StopGeneratingAll();
			mKinectStruct->mContext.Release();
			//mKinectStruct->mContext.Shutdown();
		}

		KINECT::~KINECT()
		{
			mKinectStruct->mContext.StopGeneratingAll();
			mKinectStruct->mContext.Release();
			// mKinectStruct->mContext.Shutdown();
			cout<<"Device Close!"<<endl;
		}

		void KINECT::updateData(VISION_DATA &data)
		{
			mKinectStruct->mStatus = mKinectStruct->mContext.WaitAndUpdateAll();
			//mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "View Cloud");

			const XnDepthPixel* pDepthMap = mKinectStruct->mDepthGenerator.GetDepthMap();

			mKinectStruct->v1PointCloud.clear();
			mKinectStruct->v2PointCloud.clear();

			GeneratePointCloud(mKinectStruct->mDepthGenerator, pDepthMap, mKinectStruct->v1PointCloud);

			double kinectAdjust[4][4] = {{-1, 0, 0, 0},
										 {0, 1, 0, 0},
										 {0, 0, 1, 0},
										 {0, 0, 0, 1}};

			double kinectToRobot[4][4] = {{0.9995, 0.0134, -0.0273, 0.0224},
										  {-0.0304, 0.5120, -0.8584, 0.2026 + 0.038},
										  {0.0025, 0.8589, 0.5122, 0.5733},
										  {0, 0, 0, 1}};

			double robotToWorld[4][4] = {{1, 0, 0, 0},
										 {0, 1, 0, 0},
										 {0, 0, 1, 0},
										 {0, 0, 0, 1}};

			double kinectToWorld[4][4], tempMatrix[4][4];

			MatrixMultiple(kinectToRobot, kinectAdjust, tempMatrix);

			MatrixMultiple(robotToWorld, tempMatrix, kinectToWorld);

			for(int i = 0; i < mKinectStruct->v1PointCloud.size(); ++i)
			{
				Point3D tempPoint;
				tempPoint.X = kinectToWorld[0][0]*mKinectStruct->v1PointCloud[i].X + kinectToWorld[0][1]*mKinectStruct->v1PointCloud[i].Y
						+ kinectToWorld[0][2]*mKinectStruct->v1PointCloud[i].Z + kinectToWorld[0][3];

				tempPoint.Y = kinectToWorld[1][0]*mKinectStruct->v1PointCloud[i].X + kinectToWorld[1][1]*mKinectStruct->v1PointCloud[i].Y
						+ kinectToWorld[1][2]*mKinectStruct->v1PointCloud[i].Z + kinectToWorld[1][3];

				tempPoint.Z = kinectToWorld[2][0]*mKinectStruct->v1PointCloud[i].X + kinectToWorld[2][1]*mKinectStruct->v1PointCloud[i].Y
						+ kinectToWorld[2][2]*mKinectStruct->v1PointCloud[i].Z + kinectToWorld[2][3];

				mKinectStruct->v2PointCloud.push_back(tempPoint);
			}

			GenerateGridMap(data, mKinectStruct->v2PointCloud);
		}
	}
}
