#ifndef ARIS_SENSOR_BASE_H_
#define ARIS_SENSOR_BASE_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>

namespace aris
{
	/// \brief 传感器命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace sensor
	{
		template<class DataType> class SensorBase;
		
		template<class DataType>
		class SensorData
		{
		public:
			virtual ~SensorData() = default;
			SensorData() : sensor_(nullptr), data_(nullptr)
			{
			};
			explicit SensorData(SensorBase<DataType> *sensor): sensor_(sensor), data_(nullptr)
			{
				
				//这里data_to_read_指向最新的内存，例如如果data_to_read_为2，那么有两种情况：
				//1.此时正在写内存0，内存1空闲。
				//2.在某些极端特殊时刻下，sensor正好刚刚写到内存1，正准备释放dataMutex0，并且之后准备将data_to_read_置为0。
				//无论以上哪种情况，dataMutex2都会被锁住。
				//紧接着以上两种情况，继而会发生以下情况：
				//1.正在写内存0，内存1空闲，dataMutex2都会被锁住后data_to_read_依然为2，那么此后数据一直在操作内存2，安全。
				//2.dataMutex2被锁住的同时，data_to_read_被更新到0，此时传感器开始写内存1，由于dataMutex2被锁，因此传感器一直无法
				//更新到内存2；但是数据读取的是内存0，安全。
				while (!lock_.owns_lock())
				{
					lock_ = std::unique_lock<std::recursive_mutex>(sensor_->data_mutex_[sensor_->data_to_read_], std::try_to_lock);
				}
				//lock_ = std::unique_lock<std::recursive_mutex>(pSensor->data_mutex_[pSensor->data_to_read_]);
				data_ = &sensor_->data_[sensor_->data_to_read_];
			};
			SensorData(SensorData<DataType> && other) { this->swap(other); };
			SensorData & operator=(SensorData<DataType> && other) { this->swap(other); return *this; };
			auto swap(SensorData<DataType> &other)->void
			{
				std::swap(this->sensor_, other.sensor_);
				std::swap(this->data_, other.data_);
				std::swap(this->lock_, other.lock_);
			}
			auto get() const->const DataType &{ return *data_; };

		private:
			SensorData(const SensorData<DataType>&) = delete;
			SensorData & operator=(const SensorData<DataType>&) = delete;

			SensorBase<DataType> *sensor_;
			const DataType *data_;
			std::unique_lock<std::recursive_mutex> lock_;
		};
		
		template <class DataType>
		class SensorBase
		{
		public:
			virtual ~SensorBase() { stop(); };
			SensorBase() = default;
			auto start()->void
			{
				if (!sensor_thread_.joinable())
				{
					this->init();
					
					std::lock(data_mutex_[0], data_mutex_[1], data_mutex_[2]);

					std::unique_lock<std::recursive_mutex> lock1(data_mutex_[0], std::adopt_lock);
					std::unique_lock<std::recursive_mutex> lock2(data_mutex_[1], std::adopt_lock);
					std::unique_lock<std::recursive_mutex> lock3(data_mutex_[2], std::adopt_lock);
					
					for (auto &d: data_)
					{
						this->updateData(d);
					}
					
					is_stopping_ = false;
					data_to_read_ = 2;

					this->sensor_thread_ = std::thread(SensorBase::update_thread, this);
				}
			};
			auto stop()->void
			{
				if (sensor_thread_.joinable())
				{
					is_stopping_ = true;
					sensor_thread_.join();
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
			static auto update_thread(SensorBase *pSensor)->void
			{
				std::unique_lock<std::recursive_mutex> lock_(pSensor->data_mutex_[0]);
				
				while (!pSensor->is_stopping_)
				{
					for (int i = 0; i < 3; ++i)
					{
						pSensor->data_to_read_ = (i + 2) % 3;
						pSensor->updateData(pSensor->data_[i]);
						std::unique_lock<std::recursive_mutex> nextLock(pSensor->data_mutex_[(i + 1) % 3]);
						lock_.swap(nextLock);
					}
				}
			};

		private:
			std::atomic_bool is_stopping_;
			std::thread sensor_thread_;
			std::atomic_int data_to_read_;
			std::recursive_mutex data_mutex_[3];
			DataType data_[3];
			
			friend class SensorData<DataType>;
		};
	}
}


#endif
