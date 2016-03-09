#ifndef ARIS_SENSOR_BASE_H_
#define ARIS_SENSOR_BASE_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

namespace Aris
{
	/// \brief 传感器命名空间
	/// \ingroup Aris
	/// 
	///
	///
	namespace Sensor
	{
		template<class DataType> class SensorBase;
		
		template<class DataType>
		class SensorData
		{
		public:
			virtual ~SensorData() = default;
			SensorData() : pSensor(nullptr), pData(nullptr)
			{
			};
			explicit SensorData(SensorBase<DataType> *pSensor): pSensor(pSensor), pData(nullptr)
			{
				
				//这里dataToBeRead指向最新的内存，例如如果dataToBeRead为2，那么有两种情况：
				//1.此时正在写内存0，内存1空闲。
				//2.在某些极端特殊时刻下，sensor正好刚刚写到内存1，正准备释放dataMutex0，并且之后准备将dataToBeRead置为0。
				//无论以上哪种情况，dataMutex2都会被锁住。
				//紧接着以上两种情况，继而会发生以下情况：
				//1.正在写内存0，内存1空闲，dataMutex2都会被锁住后dataToBeRead依然为2，那么此后数据一直在操作内存2，安全。
				//2.dataMutex2被锁住的同时，dataToBeRead被更新到0，此时传感器开始写内存1，由于dataMutex2被锁，因此传感器一直无法
				//更新到内存2；但是数据读取的是内存0，安全。
				while (!lock.owns_lock())
				{
					lock = std::unique_lock<std::recursive_mutex>(pSensor->dataMutex[pSensor->dataToBeRead], std::try_to_lock);
				}
				//lock = std::unique_lock<std::recursive_mutex>(pSensor->dataMutex[pSensor->dataToBeRead]);
				pData = &pSensor->data[pSensor->dataToBeRead];
			};
			SensorData(SensorData<DataType> && other) { this->swap(other); };
			SensorData & operator=(SensorData<DataType> && other) { this->swap(other); return *this; };
			auto swap(SensorData<DataType> &other)->void
			{
				std::swap(this->pSensor, other.pSensor);
				std::swap(this->pData, other.pData);
				std::swap(this->lock, other.lock);
			}
			auto get() const->const DataType &{ return *pData; };

		private:
			SensorData(const SensorData<DataType>&) = delete;
			SensorData & operator=(const SensorData<DataType>&) = delete;

			SensorBase<DataType> *pSensor;
			const DataType *pData;
			std::unique_lock<std::recursive_mutex> lock;
		};
		
		template <class DataType>
		class SensorBase
		{
		public:
			virtual ~SensorBase() { stop(); };
			SensorBase() = default;
			auto start()->void
			{
				if (!sensor_thread.joinable())
				{
					this->init();
					
					std::lock(dataMutex[0], dataMutex[1], dataMutex[2]);

					std::unique_lock<std::recursive_mutex> lock1(dataMutex[0], std::adopt_lock);
					std::unique_lock<std::recursive_mutex> lock2(dataMutex[1], std::adopt_lock);
					std::unique_lock<std::recursive_mutex> lock3(dataMutex[2], std::adopt_lock);
					
					for (auto &d:data)
					{
						this->updateData(d);
					}
					
					isStoping = false;
					dataToBeRead = 2;

					this->sensor_thread = std::thread(SensorBase::update_thread, this);
				}
			};
			auto stop()->void
			{
				if (sensor_thread.joinable())
				{
					isStoping = true;
					sensor_thread.join();
					this->release();
				}
			};
			auto getSensorData()->SensorData<DataType> { return std::move(SensorData<DataType>(this)); };

		private:
			SensorBase(const SensorBase&) = delete;
			SensorBase & operator=(const SensorBase&) = delete;
			virtual auto init()->void {};
			virtual auto release()->void {};
			virtual auto updateData(DataType &data)->void = 0;
			static void update_thread(SensorBase *pSensor)
			{
				std::unique_lock<std::recursive_mutex> lock(pSensor->dataMutex[0]);
				
				while (!pSensor->isStoping)
				{
					for (int i = 0; i < 3; ++i)
					{
						pSensor->dataToBeRead = (i + 2) % 3;
						pSensor->updateData(pSensor->data[i]);
						std::unique_lock<std::recursive_mutex> nextLock(pSensor->dataMutex[(i + 1) % 3]);
						lock.swap(nextLock);
					}
				}
			};

		private:
			std::atomic_bool isStoping;
			std::thread sensor_thread;
			std::atomic_int dataToBeRead;
			std::recursive_mutex dataMutex[3];
			DataType data[3];
			
			friend class SensorData<DataType>;
		};
	}
}


#endif
