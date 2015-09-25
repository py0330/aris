#ifndef ARIS_SENSOR_H_
#define ARIS_SENSOR_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

namespace Aris
{
	namespace Sensor
	{
		template<class DATA_TYPE> class SENSOR_BASE;
		
		template<class DATA_TYPE>
		class SENSOR_DATA
		{
		public:
			const DATA_TYPE &Get() const { return *pData; };
			explicit SENSOR_DATA(SENSOR_BASE<DATA_TYPE> *pSensor): pSensor(pSensor), pData(nullptr)
			{
				/*
				这里dataToBeRead指向最新的内存，例如如果dataToBeRead为2，那么有两种情况：
				1.此时正在写内存0，内存1空闲。
				2.在某些极端特殊时刻下，sensor正好刚刚写到内存1，正准备释放dataMutex0，并且之后准备将dataToBeRead置为0。
				无论以上哪种情况，dataMutex2都会被锁住。
				紧接着以上两种情况，继而会发生以下情况：
				1.正在写内存0，内存1空闲，dataMutex2都会被锁住后dataToBeRead依然为2，那么此后数据一直在操作内存2，安全。
				2.dataMutex2被锁住的同时，dataToBeRead被更新到0，此时传感器开始写内存1，由于dataMutex2被锁，因此传感器一直无法
				更新到内存2；但是数据读取的是内存0，安全。
				*/
				while (!lock.owns_lock())
				{
					lock = std::unique_lock<std::recursive_mutex>(pSensor->dataMutex[pSensor->dataToBeRead], std::try_to_lock);
				}
				//lock = std::unique_lock<std::recursive_mutex>(pSensor->dataMutex[pSensor->dataToBeRead]);
				pData = &pSensor->data[pSensor->dataToBeRead];
			};
			SENSOR_DATA(SENSOR_DATA<DATA_TYPE> && other) { this->Swap(other); };
			virtual ~SENSOR_DATA() = default;
			void Swap(SENSOR_DATA<DATA_TYPE> &other)
			{
				std::swap(this->pSensor, other.pSensor);
				std::swap(this->pData, other.pData);
				std::swap(this->lock, other.lock);
			}

		private:
			SENSOR_DATA(const SENSOR_DATA&) = delete;
			SENSOR_DATA & operator=(const SENSOR_DATA&) = delete;

			SENSOR_BASE<DATA_TYPE> *pSensor;
			const DATA_TYPE *pData;
			std::unique_lock<std::recursive_mutex> lock;
		};
		
		class SENSOR_INTERFACE
		{
		public:
			virtual void Start() = 0;
			virtual void Stop() = 0;
			SENSOR_INTERFACE() = default;
			virtual ~SENSOR_INTERFACE() = default;

		private:
			SENSOR_INTERFACE(const SENSOR_INTERFACE&) = delete;
			SENSOR_INTERFACE & operator=(const SENSOR_INTERFACE&) = delete;
		};

		template <class DATA_TYPE>
		class SENSOR_BASE:public SENSOR_INTERFACE
		{
		public:
			virtual void Start() final
			{
				if (!sensor_thread.joinable())
				{
					this->Initiate();
					
					std::lock(dataMutex[0], dataMutex[1], dataMutex[2]);

					std::unique_lock<std::recursive_mutex> lock1(dataMutex[0], std::adopt_lock);
					std::unique_lock<std::recursive_mutex> lock2(dataMutex[1], std::adopt_lock);
					std::unique_lock<std::recursive_mutex> lock3(dataMutex[2], std::adopt_lock);
					
					for (auto &d:data)
					{
						this->UpdateData(d);
					}
					
					isStoping = false;
					dataToBeRead = 2;

					this->sensor_thread = std::thread(SENSOR_BASE::update_thread, this);
				}
			};
			virtual void Stop() final
			{
				if (sensor_thread.joinable())
				{
					isStoping = true;
					sensor_thread.join();
					this->Release();
				}
			};
			virtual SENSOR_DATA<DATA_TYPE> GetSensorData()
			{
				return std::move(SENSOR_DATA<DATA_TYPE>(this));
			};

			SENSOR_BASE() = default;
			virtual ~SENSOR_BASE() { Stop(); };

		private:
			virtual void Initiate() {};
			virtual void Release() {};
			virtual void UpdateData(DATA_TYPE &data) = 0;

			SENSOR_BASE(const SENSOR_BASE&) = delete;
			SENSOR_BASE & operator=(const SENSOR_BASE&) = delete;

			static void update_thread(SENSOR_BASE *pSensor)
			{
				std::unique_lock<std::recursive_mutex> lock(pSensor->dataMutex[0]);
				
				while (!pSensor->isStoping)
				{
					for (int i = 0; i < 3; ++i)
					{
						pSensor->dataToBeRead = (i + 2) % 3;
						pSensor->UpdateData(pSensor->data[i]);
						std::unique_lock<std::recursive_mutex> nextLock(pSensor->dataMutex[(i+1)%3]);
						lock.swap(nextLock);
					}
				}
			};
		private:
			std::atomic_bool isStoping;
			std::thread sensor_thread;
			std::atomic_int dataToBeRead;
			std::recursive_mutex dataMutex[3];
			DATA_TYPE data[3];
			
			friend class SENSOR_DATA<DATA_TYPE>;
		};
	}
}


#endif
