#ifndef ARIS_VISION_H_
#define ARIS_VISION_H_

#include "Aris_Sensor.h"
#include <memory>
#include <stdlib.h>
#include <iostream>

using namespace std;

namespace Aris
{
	namespace Sensor
	{

		struct VISION_DATA
		{
			std::int64_t timeStamp;
			double gridMap[120][120];
			friend class KINECT;
		};

		class KINECT: public SENSOR_BASE<VISION_DATA>
		{

		public:
			KINECT();
			~KINECT();

		private:
			virtual void Initiate();
			virtual void UpdateData(VISION_DATA &data);
			virtual void Release();

		private:
			class KINECT_STRUCT;
			std::auto_ptr<KINECT_STRUCT> mKinectStruct;
		};
	}
}

#endif // ARIS_VISION_H_
