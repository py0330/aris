#ifndef ARIS_IMU_H_
#define ARIS_IMU_H_

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cstdint>

#include "aris_xml.h"
#include "aris_sensor.h"


namespace Aris
{
	namespace Sensor
	{
		struct ImuData
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

			void ToPmBody2Ground(double *pm) const;
			void ToPmBody2Ground(double *pm, double yawValue) const;
			void ToEulBody2Ground(double *eul, const char *eulType="313") const;
			void ToEulBody2Ground(double *eul, double yawValue, const char *eulType="313") const;
			void ToOmegaBody2Ground(double *omega) const;
			void ToOmegaBody2Ground(double *omega, double yawValue) const;
			void ToPntAccBody2Ground(double *acc) const;
			void ToPntAccBody2Ground(double *acc, double yawValue) const;

		private:
			const double * pmLhs;
			const double * pmRhs;

			friend class IMU;
		};

		class IMU :public SensorBase<ImuData>
		{
		public:
			IMU();
			IMU(const Aris::Core::XmlElement *xmlEle);
			~IMU();
		
		private:
			virtual void Init();
			virtual void Release();
			virtual void UpdateData(ImuData &data);

		private:
			class Imp;
			std::unique_ptr<Imp> pImp;
		};
	}
}


#endif