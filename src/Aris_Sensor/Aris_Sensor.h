#ifndef ARIS_SENSOR_H_
#define ARIS_SENSOR_H_

#include <memory>
#include <thread>

using namespace std;

namespace Aris
{
	namespace Sensor
	{
		/*class SENSOR_BASE
		{
		public:
			virtual int GetDataSize() const = 0;
			void Start();
			void Stop();

			SENSOR_BASE();
			virtual ~SENSOR_BASE() = default;

		private:
			virtual void UpdateData(char *dataAddress) = 0;

			SENSOR_BASE(const SENSOR_BASE&) = delete;
			SENSOR_BASE & operator=(const SENSOR_BASE&) = delete;

		private:
			std::unique_ptr<char> data1{ new char[GetDataSize()] }, data2{ new char[GetDataSize()] };
			std::thread sensor_thread;
		};*/
	}
}


#endif