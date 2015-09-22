#ifndef ARIS_IMU_H_
#define ARIS_IMU_H_

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

#include <Aris_Sensor.h>

namespace Aris
{
	namespace Sensor
	{
		struct IMU_DATA
		{
			union
			{
				double eul[3];
				struct
				{
					double a, b, c;
				};
			};

			union
			{
				double omega[3];
				struct
				{
					double va, vb, vc;
				};
			};

			union
			{
				double alpha[3];
				struct
				{
					double aa, ab, ac;
				};
			};

			union
			{
				double gravity[3];
				struct
				{
					double gx, gy, gz;
				};
			};
		};

		class IMU_IMPLEMENT;

		class IMU :public SENSOR_BASE<IMU_DATA>
		{
		public:
			IMU();
			~IMU();
		
		private:
			virtual void Initiate();
			virtual void Release();
			virtual void UpdateData(IMU_DATA &data);

		private:
			IMU_IMPLEMENT *pDevice;
		};
	}
}


#endif