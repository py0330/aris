#ifndef ARIS_IMU_H_
#define ARIS_IMU_H_

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cstdint>

#include <Aris_XML.h>
#include <Aris_Sensor.h>


namespace Aris
{
	namespace Sensor
	{
		struct IMU_DATA
		{
			/*时间戳*/
			std::int64_t time;
			
			/*原生数据，相对于陀螺仪自身定义的地面坐标系的321欧拉角*/
			union
			{
				double eul321[3];
				struct
				{
					double yaw, pitch, roll;
				};
			};
			/*原生数据，相对于陀螺仪坐标系的角速度*/
			union
			{
				double omega[3];
				struct
				{
					double va, vb, vc;
				};
			};
			/*原生数据，相对于陀螺仪坐标系的线加速度*/
			union
			{
				double acc[3];
				struct
				{
					double ax, ay, az;
				};
			};

			void ToBodyEul(double *eul, const char *eulType="313") const;
			void ToBodyEul(double *eul, double yawValue, const char *eulType="313") const;
			void ToBodyPm(double *pm) const;
			void ToBodyPm(double *pm, double yawValue) const;
			void ToBodyOmega(double *omega) const;
			void ToBodyOmega(double *omega, double yawValue) const;
			void ToBodyAcc(double *acc) const;
			void ToBodyAcc(double *acc, double yawValue) const;

		private:
			const double * pmLhs;
			const double * pmRhs;

			friend class IMU;
		};

		class IMU :public SENSOR_BASE<IMU_DATA>
		{
		public:
			IMU();
			IMU(const Aris::Core::ELEMENT *xmlEle);
			~IMU();
		
		private:
			virtual void Initiate();
			virtual void Release();
			virtual void UpdateData(IMU_DATA &data);

		private:
			class IMU_IMP;
			IMU_IMP *pDevice;
		};
	}
}


#endif