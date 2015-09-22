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
			const DATA_TYPE &Get() const
			{
				return *pData;
			};
			explicit SENSOR_DATA(SENSOR_BASE<DATA_TYPE> *pSensor): pSensor(pSensor), pData(nullptr)
			{
				for (int i = 0; i < 3; ++i)
				{
					lock[i] = std::unique_lock<std::recursive_mutex>(pSensor->dataMutex[i], std::defer_lock);
				}
				
				/*这里总是尝试去读取上次读过的内存所对应的下一块内存*/
				while (!pData)
				{
					if (lock[pSensor->dataToBeRead].try_lock())
					{
						pData = &pSensor->data[pSensor->dataToBeRead];
						pSensor->dataToBeRead = (pSensor->dataToBeRead + 1) % 3;
					}
					else if (lock[(pSensor->dataToBeRead+2)%3].try_lock())
					{
						pData = &pSensor->data[(pSensor->dataToBeRead + 2) % 3];
					}
					else
					{
						pData = &pSensor->data[(pSensor->dataToBeRead + 1) % 3];
						pSensor->dataToBeRead = (pSensor->dataToBeRead + 2) % 3;
					};
				}
			};
			SENSOR_DATA(SENSOR_DATA<DATA_TYPE> && other)
			{
				this->Swap(other);
			};
			virtual ~SENSOR_DATA()
			{
			};
			void Swap(SENSOR_DATA<DATA_TYPE> &other)
			{
				std::swap(this->pSensor, other.pSensor);
				std::swap(this->pData, other.pData);

				int i{ 0 };
				for (auto &l : lock)
				{
					l.swap(other.lock[i]);
					++i;
				}
			}

		private:
			SENSOR_DATA(const SENSOR_DATA&) = delete;
			SENSOR_DATA & operator=(const SENSOR_DATA&) = delete;

			std::unique_lock<std::recursive_mutex> lock[3];
			SENSOR_BASE<DATA_TYPE> *pSensor;
			const DATA_TYPE *pData;
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
					dataToBeRead = 0;

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
			virtual ~SENSOR_BASE() = default;

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
						pSensor->UpdateData(pSensor->data[i]);
						std::unique_lock<std::recursive_mutex> nextLock(pSensor->dataMutex[(i+1)%3]);
						lock.swap(nextLock);
					}
				}
			};
		private:
			int dataToBeRead;
			DATA_TYPE data[3];
			std::recursive_mutex dataMutex[3];
			std::atomic_bool isStoping;
			std::thread sensor_thread;

			friend class SENSOR_DATA<DATA_TYPE>;
		};
	}
}


#endif