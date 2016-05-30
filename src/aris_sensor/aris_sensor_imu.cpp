#include <string>

#include "aris_core.h"
#include "aris_dynamic.h"
#include "aris_sensor_imu.h"

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "deviceclass.h"

namespace aris
{
	namespace sensor
	{
		void ImuData::toPmBody2Ground(double *pm) const
		{
			toPmBody2Ground(pm, this->yaw);
		}
		void ImuData::toPmBody2Ground(double *pm, double yawValue) const
		{
			double tem_pm[16];
			double pe[6]{ 0,0,0,yawValue, pitch, roll };
			aris::dynamic::s_pe2pm(pe, tem_pm, "321");
			aris::dynamic::s_pm_dot_pm(pmLhs, tem_pm, pmRhs, pm);
		}
		void ImuData::toEulBody2Ground(double *eul, const char *eulType) const
		{
			this->toEulBody2Ground(eul, this->yaw, eulType);
		}
		void ImuData::toEulBody2Ground(double *eul, double yawValue, const char *eulType) const
		{
			double pm[16], pe[6];
			this->toPmBody2Ground(pm, yawValue);
			aris::dynamic::s_pm2pe(pm, pe, eulType);
			std::copy_n(pe + 3, 3, eul);
		}
		void ImuData::toOmegaBody2Ground(double *omega) const
		{
			toOmegaBody2Ground(omega, this->yaw);
		}
		void ImuData::toOmegaBody2Ground(double *omega, double yawValue) const
		{
			double pm[16];
			toPmBody2Ground(pm, yawValue);
			aris::dynamic::s_pm_dot_v3(pm, this->omega, omega);
		}
		void ImuData::toPntAccBody2Ground(double *acc) const
		{
			toPntAccBody2Ground(acc, this->yaw);
		}
		void ImuData::toPntAccBody2Ground(double *acc, double yawValue) const
		{
			/*需要角加速度才能算出准确值*/
			
			//toPntAccBody2Ground(acc, this->yaw);
		}
		
		class Imu::Imp :public DeviceClass
		{
		public:
			std::string port;
			int baudRate;
			int sampleRate;

			double pmImuGround2BodyGround[4][4];
			double pmBody2Imu[4][4];

		public:
			XsPortInfo mtPort;
		};
		
		Imu::Imu(Object &father, std::size_t id, const std::string &name)
			: SensorTemplate(father, id, name)
			, imp_(new Imp)
		{
			imp_->port = "/dev/ttyUSB0";
			imp_->baudRate = 921600;
			imp_->sampleRate = 400;

			std::fill_n(&imp_->pmImuGround2BodyGround[0][0], 16, 0);
			imp_->pmImuGround2BodyGround[0][0] = -1;
			imp_->pmImuGround2BodyGround[1][2] = 1;
			imp_->pmImuGround2BodyGround[2][1] = 1;
			imp_->pmImuGround2BodyGround[3][3] = 1;

			std::fill_n(&imp_->pmBody2Imu[0][0], 16, 0);
			imp_->pmBody2Imu[0][0] = 1;
			imp_->pmBody2Imu[1][2] = 1;
			imp_->pmBody2Imu[2][1] = -1;
			imp_->pmBody2Imu[3][3] = 1;
		}
		Imu::Imu(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele): SensorTemplate(father, id, xml_ele), imp_(new Imp)
		{
#ifdef UNIX
			imp_->port = xml_ele.Attribute("portLinux");
#endif
			imp_->baudRate = std::stoi(xml_ele.Attribute("baudRate"));
			imp_->sampleRate = std::stoi(xml_ele.Attribute("sampleRate"));;

			aris::core::Calculator c;
			c.addVariable("PI", PI);
			
			auto m = c.calculateExpression(xml_ele.Attribute("PeImuGround2BodyGound"));
			aris::dynamic::s_pe2pm(m.data(), &imp_->pmImuGround2BodyGround[0][0]);
			m = c.calculateExpression(xml_ele.Attribute("PeImu2Body"));
			double pmImu2Body[16];
			aris::dynamic::s_pe2pm(m.data(), pmImu2Body);
			aris::dynamic::s_inv_pm(pmImu2Body, &imp_->pmBody2Imu[0][0]);
		}
		Imu::~Imu()
		{
		}

		void Imu::init()
		{
			XsPortInfoArray portInfoArray;
			xsEnumerateUsbDevices(portInfoArray);

			if (!portInfoArray.size())
			{
#ifdef WIN32
				throw std::runtime_error("Imu: failed to find Imu sensor");
#endif
#ifdef UNIX
				XsPortInfo portInfo(imp_->port, XsBaud::numericToRate(imp_->baudRate));
				portInfoArray.push_back(portInfo);
#endif
			}
			
			imp_->mtPort = portInfoArray.at(0);
			
			// Open the port with the detected device
			if (!imp_->openPort(imp_->mtPort))
				throw std::runtime_error("Imu: could not open port.");
			
			aris::core::msSleep(10);

			// Put the device in configuration mode
			if (!imp_->gotoConfig()) // Put the device into configuration mode before configuring the device
			{
				throw std::runtime_error("Imu: could not put device into configuration mode");
			}
			
			// Request the device Id to check the device type
			imp_->mtPort.setDeviceId(imp_->getDeviceId());
			
			// Check if we have an MTi / MTx / MTmk4 device
			if (!imp_->mtPort.deviceId().isMtMk4())
			{
				throw std::runtime_error("Imu: No MTi / MTx / MTmk4 device found.");
			}
			
			// Check device type
			if (imp_->mtPort.deviceId().isMtMk4())
			{
				XsOutputConfiguration config0(XDI_Quaternion, imp_->sampleRate);
				XsOutputConfiguration config1(XDI_DeltaQ, imp_->sampleRate);
				XsOutputConfiguration config2(XDI_DeltaV, imp_->sampleRate);
				XsOutputConfiguration config3(XDI_Acceleration, imp_->sampleRate);

				XsOutputConfigurationArray configArray;
				configArray.push_back(config0);
				configArray.push_back(config1);
				configArray.push_back(config2);
				configArray.push_back(config3);
				if (!imp_->setOutputConfiguration(configArray))
				{
					throw std::runtime_error("Imu: Could not configure MTmk4 imp_-> Aborting.");
				}
			}
			else
			{
				throw std::runtime_error("Imu: Unknown device while configuring. Aborting.");
			}
			
			// Put the device in measurement mode
			if (!imp_->gotoMeasurement())
			{
				throw std::runtime_error("Imu: Could not put device into measurement mode. Aborting.");
			}
		}
		void Imu::release()
		{
			imp_->close();
		}
		void Imu::updateData(SensorData & data_in)
		{
			XsByteArray imuData;
			XsMessageArray msgs;

			auto &data = static_cast<ImuData&>(data_in);

			while (msgs.empty())
			{
				aris::core::msSleep(1);
				imp_->readDataToBuffer(imuData);
				imp_->processBufferedData(imuData, msgs);
			}

			//std::cout << "msg num:" << msgs.size()<<std::endl;

			for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
			{
				// Retrieve a packet
				XsDataPacket packet;
				if ((*it).getMessageId() == XMID_MtData2) 
				{
					packet.setMessage((*it));
					packet.setDeviceId(imp_->mtPort.deviceId());
				}

				// Get the all data
				auto eul = packet.orientationEuler();
				auto sdi = packet.sdiData();
				auto acc = packet.calibratedAcceleration();

				data.yaw = eul.yaw()*PI / 180;
				data.pitch = eul.pitch()*PI / 180;
				data.roll = eul.roll()*PI / 180;

				data.va = sdi.orientationIncrement().x() * 2 * 100;
				data.vb = sdi.orientationIncrement().y() * 2 * 100;
				data.vc = sdi.orientationIncrement().z() * 2 * 100;
				
				std::copy_n(acc.data(), acc.size(), data.acc);

				data.time = packet.timeOfArrival().nowMs();
				data.pmLhs = *imp_->pmImuGround2BodyGround;
				data.pmRhs = *imp_->pmBody2Imu;
			}

			msgs.clear();
		}
	}
}
