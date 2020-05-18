#ifndef ARIS_SENSOR_BASE_H_
#define ARIS_SENSOR_BASE_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>

#include <aris_lib_export.h>
#include <aris/core/core.hpp>

namespace aris::sensor
{
	class Sensor;

	struct ARIS_API SensorData {};
	class ARIS_API SensorDataProtector
	{
	public:
		auto get() const->const SensorData * { return data_; }
		auto data() const->const SensorData & { return *data_; }
		auto operator->()const -> const SensorData * { return data_; }
		auto operator*()const -> const SensorData & { return std::ref(*data_); }
		auto operator=(SensorDataProtector && other)->SensorDataProtector & { std::swap(*this, other); return *this; }

		~SensorDataProtector() = default;
		SensorDataProtector() : sensor_(nullptr), data_(nullptr) {}
		SensorDataProtector(SensorDataProtector && other) = default;

	private:
		explicit SensorDataProtector(Sensor *sensor);
		SensorDataProtector(const SensorDataProtector&) = delete;
		SensorDataProtector & operator=(const SensorDataProtector&) = delete;

		Sensor *sensor_;
		const SensorData *data_;
		std::unique_lock<std::recursive_mutex> lock_;

		friend class Sensor;
	};
	class ARIS_API Sensor
	{
	public:
		auto start()->void;
		auto stop()->void;
		auto dataProtector()->SensorDataProtector;

		virtual ~Sensor();
		Sensor(const std::string &name, std::function<SensorData*()> new_func);

	protected:
		auto virtual init()->void {}
		auto virtual release()->void {}
		auto virtual updateData(SensorData & data)->void {}

	private:
		//auto operator=(Sensor &&)->Sensor& = default;
		//Sensor(Sensor &&) = default;

		struct Imp;
		std::unique_ptr<Imp> imp_;

		friend class SensorDataProtector;
		template<class DataType> friend class SensorTemplate;
	};
	template<class DataType> class SensorTemplate :public Sensor
	{
	public:
		SensorTemplate(const std::string &name) :Sensor(name, []()->SensorData* {return new DataType; }) {}
	};

	class ARIS_API SensorRoot
	{
	public:
		//auto sensorPool()->aris::core::ObjectPool<Sensor> &;
		//auto sensorPool()const->const aris::core::ObjectPool<Sensor> &;
		//auto start()->void { for (auto &sensor : sensorPool())sensor.start(); }
		//auto stop()->void { for (auto &sensor : sensorPool())sensor.stop(); }

		virtual ~SensorRoot();
		explicit SensorRoot(const std::string &name = "sensor_root");
		ARIS_DECLARE_BIG_FOUR(SensorRoot);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}


#endif
