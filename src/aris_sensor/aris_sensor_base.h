#ifndef ARIS_SENSOR_BASE_H_
#define ARIS_SENSOR_BASE_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>

#include <aris_core.h>

namespace aris
{
	/// \brief 传感器命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace sensor
	{
		class Sensor;
		
		struct SensorData {};
		class SensorDataProtector 
		{
		public:
			auto get() const->const SensorData *{ return data_; };
			auto data() const->const SensorData &{ return *data_; };
			auto operator->()const -> const SensorData *{ return data_; };
			auto operator*()const -> const SensorData &{ return std::ref(*data_); };
			auto operator=(SensorDataProtector && other)->SensorDataProtector & { std::swap(*this, other); return *this; };

			~SensorDataProtector() = default;
			SensorDataProtector() : sensor_(nullptr), data_(nullptr) {};
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
		class Sensor :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("sensor"); return std::ref(type); };
			virtual auto type() const->const std::string&{ return Type(); };
			auto start()->void;
			auto stop()->void;
			auto dataProtector()->SensorDataProtector;

			virtual ~Sensor();
			Sensor(Object &father, std::size_t id, const std::string &name, std::function<SensorData*()> new_func);
			Sensor(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele, std::function<SensorData*()> new_func);

		protected:
			virtual auto init()->void {};
			virtual auto release()->void {};
			virtual auto updateData(SensorData & data)->void {};

		private:
			auto operator=(const Sensor &)->Sensor& = default;
			auto operator=(Sensor &&)->Sensor& = default;
			Sensor(const Sensor &) = default;
			Sensor(Sensor &&) = default;

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class SensorDataProtector;
			template<class DataType> friend class SensorTemplate;
		};
		template<class DataType> class SensorTemplate :public Sensor
		{
		public:
			SensorTemplate(Object &father, std::size_t id, const std::string &name) :Sensor(father, id, name, []()->SensorData* {return new DataType; }) {};
			SensorTemplate(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :Sensor(father, id, xml_ele, []()->SensorData* {return new DataType; }) {};
		};

		class SensorRoot:public aris::core::Root
		{
		public:
			using Root::loadXml;
			virtual auto loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto sensorPool()->aris::core::ObjectPool<Sensor> &;
			auto sensorPool()const->const aris::core::ObjectPool<Sensor> &;
			auto start()->void { for (auto &sensor : sensorPool())sensor.start(); };
			auto stop()->void { for (auto &sensor : sensorPool())sensor.stop(); };

			virtual ~SensorRoot();
			SensorRoot(const std::string &name = "SensorRoot");

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
	}
}


#endif
