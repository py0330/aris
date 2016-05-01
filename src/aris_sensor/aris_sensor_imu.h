#ifndef ARIS_SENSOR_IMU_H_
#define ARIS_SENSOR_IMU_H_

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cstdint>

#include <aris_core.h>
#include <aris_sensor_base.h>

namespace aris
{
	namespace sensor
	{
		struct ImuData:public SensorData
		{
			/// 时间戳 ///
			std::int64_t time;
			
			/// 原生数据，相对于陀螺仪自身定义的地面坐标系的321欧拉角 ///
			union
			{
				double eul321[3];
				struct
				{
					double yaw, pitch, roll;
				};
			};
			/// 原生数据，相对于陀螺仪坐标系的角速度 ///
			union
			{
				double omega[3];
				struct
				{
					double va, vb, vc;
				};
			};
			/// 原生数据，相对于陀螺仪坐标系的线加速度 ///
			union
			{
				double acc[3];
				struct
				{
					double ax, ay, az;
				};
			};

			auto toPmBody2Ground(double *pm) const->void;
			auto toPmBody2Ground(double *pm, double yawValue) const->void;
			auto toEulBody2Ground(double *eul, const char *eulType="313") const->void;
			auto toEulBody2Ground(double *eul, double yawValue, const char *eulType="313") const->void;
			auto toOmegaBody2Ground(double *omega) const->void;
			auto toOmegaBody2Ground(double *omega, double yawValue) const->void;
			auto toPntAccBody2Ground(double *acc) const->void;
			auto toPntAccBody2Ground(double *acc, double yawValue) const->void;

		private:
			const double * pmLhs;
			const double * pmRhs;

			friend class Imu;
		};
		class Imu :public SensorTemplate<ImuData>
		{
		public:
			~Imu();
			Imu(Object &father, std::size_t id, const std::string &name);
			Imu(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			
		private:
			virtual auto init()->void final override;
			virtual auto release()->void final override;
			virtual auto updateData(SensorData & data_in)->void final override;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;
		};
	}
}


#endif