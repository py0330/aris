#ifndef ARIS_IMU_H_
#define ARIS_IMU_H_

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cstdint>

#include <Aris_Sensor.h>

namespace Aris
{
	namespace Sensor
	{
		struct IMU_DATA
		{
			std::int64_t time;
			
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
			class IMU_IMPLEMENT;
			IMU_IMPLEMENT *pDevice;
		};
	}
}


#endif