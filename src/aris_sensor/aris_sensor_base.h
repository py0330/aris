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
			auto start()->void;
			auto stop()->void;
			auto dataProtector()->SensorDataProtector;

			virtual ~Sensor();
			Sensor(Object &father, std::size_t id, const std::string &name, std::function<SensorData*()> new_func);
			Sensor(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele, std::function<SensorData*()> new_func);

		private:
			virtual auto init()->void {};
			virtual auto release()->void {};
			virtual auto updateData(SensorData & data)->void {};

			auto operator=(const Sensor &)->Sensor& = delete;
			auto operator=(Sensor &&)->Sensor& = delete;
			Sensor(const Sensor &) = delete;
			Sensor(Sensor &&) = delete;

			struct Imp;
			std::unique_ptr<Imp> imp_;

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
			class iterator
			{
			public:
				typedef Root::difference_type difference_type;
				typedef Sensor value_type;
				typedef Sensor& reference;
				typedef Sensor* pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				iterator() = default;
				iterator(const iterator& other) = default;
				iterator(const Root::iterator iter) :iter_(iter) {}; // 自己添加的
				~iterator() = default;

				auto operator=(const iterator&other)->iterator& = default;
				auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; };
				auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; };
				auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; }; //optional
				auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; }; //optional
				auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; }; //optional
				auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; }; //optional

				auto operator++()->iterator& { ++iter_; return *this; };
				auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; }; //optional
				auto operator--()->iterator& { --iter_; return *this; }; //optional
				auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; }; //optional
				auto operator+=(size_type size)->iterator& { iter_ += size; return *this; }; //optional
				auto operator+(size_type size) const->iterator { return iterator(iter_ + size); }; //optional
				friend auto operator+(size_type size, const iterator&iter_)->iterator { return iterator(size + iter_); }; //optional
				auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; }; //optional
				auto operator-(size_type size) const->iterator { return iter_ - size; }; //optional
				auto operator-(iterator iter) const->difference_type { return this->iter_ - iter.iter_; }; //optional

				auto operator*() const->reference { return static_cast<reference>(iter_.operator*()); };
				auto operator->() const->pointer { return static_cast<pointer>(iter_.operator->()); };
				auto operator[](size_type size) const->reference { return static_cast<reference>(iter_->operator[](size)); }; //optional

				aris::core::Root::iterator iter_;
				friend class const_iterator;
			};
			class const_iterator
			{
			public:
				typedef Root::difference_type difference_type;
				typedef Sensor value_type;
				typedef const Sensor& const_reference;
				typedef const Sensor* const_pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				const_iterator() = default;
				const_iterator(const const_iterator&) = default;
				const_iterator(const iterator& other) :iter_(other.iter_) {};
				const_iterator(const Root::const_iterator iter) :iter_(iter) {}; // 自己添加的
				~const_iterator() = default;

				auto operator=(const const_iterator&)->const_iterator& = default;
				auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; };
				auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; };
				auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; }; //optional
				auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; }; //optional
				auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; }; //optional
				auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; }; //optional

				auto operator++()->const_iterator& { ++iter_; return *this; };
				auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; };  //optional
				auto operator--()->const_iterator& { --iter_; return *this; }; //optional
				auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; }; //optional
				auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; }; //optional
				auto operator+(size_type size) const->const_iterator { return const_iterator(iter_ + size); }; //optional
				friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return const_iterator(size + iter); }; //optional
				auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; }; //optional
				auto operator-(size_type size) const->const_iterator { return const_iterator(iter_ - size); }; //optional
				auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; }; //optional

				auto operator*() const->const_reference { return static_cast<const_reference>(*iter_); };
				auto operator->() const->const_pointer { return static_cast<const_pointer>(iter_.operator->()); };
				auto operator[](size_type size) const->const_reference { return static_cast<const_reference>(iter_->operator[](size)); }; //optional

				Root::const_iterator iter_;
			};

			auto begin()->iterator { return Root::begin(); };
			auto begin()const->const_iterator { return Root::begin(); };
			auto end()->iterator { return Root::end(); };
			auto end()const->const_iterator { return Root::end(); };
			auto at(std::size_t id) const->const Sensor&{ return static_cast<const Sensor&>(Root::at(id)); };
			auto at(std::size_t id)->Sensor& { return static_cast<Sensor&>(Root::at(id)); };
			auto findByName(const std::string &name)const->const_iterator { return Root::findByName(name); };
			auto findByName(const std::string &name)->iterator { return Root::findByName(name); };
			auto start()->void { for (auto &sensor : *this)sensor.start(); };
			auto stop()->void { for (auto &sensor : *this)sensor.stop(); };
		};
	}
}


#endif
