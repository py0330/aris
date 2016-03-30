#ifndef ARIS_VISION_H_
#define ARIS_VISION_H_

#include "aris_sensor.h"
#include <memory>
#include <stdlib.h>
#include <iostream>

using namespace std;

namespace aris
{
	namespace sensor
	{

		struct VISION_DATA
		{
			std::int64_t timeStamp;
			double gridMap[120][120];
			double depth[640][480];
			double pointCloud[640][480][3];
			friend class KINECT;
		};

		class KINECT: public SensorBase<VISION_DATA>
		{

		public:
			KINECT();
			~KINECT();

		private:
			virtual void init();
			virtual void updateData(VISION_DATA &data);
			virtual void release();

		private:
			double kinectToRobot[4][4];



			class KINECT_STRUCT;
			std::auto_ptr<KINECT_STRUCT> mKinectStruct;
		};
	}
}

#endif // ARIS_VISION_H_
