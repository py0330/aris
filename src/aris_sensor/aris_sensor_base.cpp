#include"aris_sensor.h"

namespace aris::sensor
{
	struct Sensor::Imp
	{
		std::atomic_bool is_stopping_;
		std::thread sensor_thread_;
		std::atomic_int data_to_read_;
		std::recursive_mutex data_mutex_[3];
		std::unique_ptr<SensorData> data_[3];

		static auto update_thread(Sensor *sensor)->void
		{
			std::unique_lock<std::recursive_mutex> lock_(sensor->imp_->data_mutex_[0]);

			while (!sensor->imp_->is_stopping_)
			{
				for (int i = 0; i < 3; ++i)
				{
					sensor->imp_->data_to_read_ = (i + 2) % 3;
					sensor->updateData(std::ref(*sensor->imp_->data_[i]));
					std::unique_lock<std::recursive_mutex> nextLock(sensor->imp_->data_mutex_[(i + 1) % 3]);
					lock_.swap(nextLock);
				}
			}
		};
	};
	auto Sensor::start()->void
	{
		if (!imp_->sensor_thread_.joinable())
		{
			init();

			std::lock(imp_->data_mutex_[0], imp_->data_mutex_[1], imp_->data_mutex_[2]);

			std::unique_lock<std::recursive_mutex> lock1(imp_->data_mutex_[0], std::adopt_lock);
			std::unique_lock<std::recursive_mutex> lock2(imp_->data_mutex_[1], std::adopt_lock);
			std::unique_lock<std::recursive_mutex> lock3(imp_->data_mutex_[2], std::adopt_lock);

			for (auto &d : imp_->data_)
			{
				this->updateData(std::ref(*d));
			}

			imp_->is_stopping_ = false;
			imp_->data_to_read_ = 2;

			imp_->sensor_thread_ = std::thread(Imp::update_thread, this);
		}
	};
	auto Sensor::stop()->void
	{
		if (imp_->sensor_thread_.joinable())
		{
			imp_->is_stopping_ = true;
			imp_->sensor_thread_.join();
			release();
		}
	};
	auto Sensor::dataProtector()->SensorDataProtector
	{
		return std::move(SensorDataProtector(this));
	}
	Sensor::~Sensor() {}
	Sensor::Sensor(const std::string &name, std::function<SensorData*()> new_func) :Object(name), imp_(new Imp)
	{
		for (auto i = 0; i < 3; ++i)imp_->data_[i].reset(new_func());
	};

	SensorDataProtector::SensorDataProtector(Sensor *sensor) : sensor_(sensor), data_(nullptr)
	{
		//这里data_to_read_指向最新的内存,例如如果data_to_read_为2,那么有两种情况：
		//1.此时正在写内存0,内存1空闲。
		//2.在某些极端特殊时刻下,sensor正好刚刚写到内存1,正准备释放dataMutex0,并且之后准备将data_to_read_置为0。
		//无论以上哪种情况,dataMutex2都会被锁住。
		//紧接着以上两种情况,继而会发生以下情况：
		//1.正在写内存0,内存1空闲,dataMutex2都会被锁住后data_to_read_依然为2,那么此后数据一直在操作内存2,安全。
		//2.dataMutex2被锁住的同时,data_to_read_被更新到0,此时传感器开始写内存1,由于dataMutex2被锁,因此传感器一直无法
		//更新到内存2；但是数据读取的是内存0,安全。

		do {
			lock_ = std::unique_lock<std::recursive_mutex>(sensor_->imp_->data_mutex_[sensor_->imp_->data_to_read_], std::try_to_lock);
		} while (!lock_.owns_lock());

		data_ = sensor_->imp_->data_[sensor_->imp_->data_to_read_].get();
	};

	struct SensorRoot::Imp
	{
		aris::core::ObjectPool<Sensor>* sensor_pool_;
	};
	auto SensorRoot::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Object::loadXml(xml_ele);
		imp_->sensor_pool_ = findByName("sensor_pool") == children().end() ? &add<aris::core::ObjectPool<Sensor> >("sensor_pool") : static_cast<aris::core::ObjectPool<Sensor> *>(&(*findByName("sensor_pool")));
	}
	auto SensorRoot::sensorPool()->aris::core::ObjectPool<Sensor>& { return *imp_->sensor_pool_; }
	auto SensorRoot::sensorPool()const->const aris::core::ObjectPool<Sensor> & { return *imp_->sensor_pool_; }
	SensorRoot::~SensorRoot() {}
	SensorRoot::SensorRoot(const std::string &name) :Object(name)
	{
		registerType<aris::core::ObjectPool<Sensor> >();
		imp_->sensor_pool_ = &add<aris::core::ObjectPool<Sensor> >("sensor_pool");
	}
	SensorRoot::SensorRoot(const SensorRoot &) = default;
	SensorRoot::SensorRoot(SensorRoot &&) = default;
	SensorRoot& SensorRoot::operator=(const SensorRoot &) = default;
	SensorRoot& SensorRoot::operator=(SensorRoot &&) = default;
}
