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
		
		class IMU::Imp :public DeviceClass
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
		
		IMU::IMU():pImp(new Imp)
		{
			pImp->port = "/dev/ttyUSB0";
			pImp->baudRate = 921600;
			pImp->sampleRate = 400;

			std::fill_n(&pImp->pmImuGround2BodyGround[0][0], 16, 0);
			pImp->pmImuGround2BodyGround[0][0] = -1;
			pImp->pmImuGround2BodyGround[1][2] = 1;
			pImp->pmImuGround2BodyGround[2][1] = 1;
			pImp->pmImuGround2BodyGround[3][3] = 1;

			std::fill_n(&pImp->pmBody2Imu[0][0], 16, 0);
			pImp->pmBody2Imu[0][0] = 1;
			pImp->pmBody2Imu[1][2] = 1;
			pImp->pmBody2Imu[2][1] = -1;
			pImp->pmBody2Imu[3][3] = 1;
		}
		IMU::IMU(const aris::core::XmlElement *xmlEle) :pImp(new Imp)
		{
#ifdef UNIX
			pImp->port = xmlEle->Attribute("portLinux");
#endif
			pImp->baudRate = std::stoi(xmlEle->Attribute("baudRate"));
			pImp->sampleRate = std::stoi(xmlEle->Attribute("sampleRate"));;

			aris::core::Calculator c;
			c.addVariable("PI", PI);
			
			auto m = c.calculateExpression(xmlEle->Attribute("PeImuGround2BodyGound"));
			aris::dynamic::s_pe2pm(m.data(), &pImp->pmImuGround2BodyGround[0][0]);
			m = c.calculateExpression(xmlEle->Attribute("PeImu2Body"));
			double pmImu2Body[16];
			aris::dynamic::s_pe2pm(m.data(), pmImu2Body);
			aris::dynamic::s_inv_pm(pmImu2Body, &pImp->pmBody2Imu[0][0]);

		}
		IMU::~IMU()
		{
		}

		void IMU::init()
		{
			XsPortInfoArray portInfoArray;
			xsEnumerateUsbDevices(portInfoArray);

			if (!portInfoArray.size())
			{
#ifdef WIN32
				throw std::runtime_error("IMU: failed to find IMU sensor");
#endif
#ifdef UNIX
				XsPortInfo portInfo(pImp->port, XsBaud::numericToRate(pImp->baudRate));
				portInfoArray.push_back(portInfo);
#endif
			}
			
			pImp->mtPort = portInfoArray.at(0);
			
			// Open the port with the detected device
			if (!pImp->openPort(pImp->mtPort))
				throw std::runtime_error("IMU: could not open port.");
			
			aris::core::msSleep(10);

			// Put the device in configuration mode
			if (!pImp->gotoConfig()) // Put the device into configuration mode before configuring the device
			{
				throw std::runtime_error("IMU: could not put device into configuration mode");
			}
			
			// Request the device Id to check the device type
			pImp->mtPort.setDeviceId(pImp->getDeviceId());
			
			// Check if we have an MTi / MTx / MTmk4 device
			if (!pImp->mtPort.deviceId().isMtMk4())
			{
				throw std::runtime_error("IMU: No MTi / MTx / MTmk4 device found.");
			}
			
			// Check device type
			if (pImp->mtPort.deviceId().isMtMk4())
			{
				XsOutputConfiguration config0(XDI_Quaternion, pImp->sampleRate);
				XsOutputConfiguration config1(XDI_DeltaQ, pImp->sampleRate);
				XsOutputConfiguration config2(XDI_DeltaV, pImp->sampleRate);
				XsOutputConfiguration config3(XDI_Acceleration, pImp->sampleRate);

				XsOutputConfigurationArray configArray;
				configArray.push_back(config0);
				configArray.push_back(config1);
				configArray.push_back(config2);
				configArray.push_back(config3);
				if (!pImp->setOutputConfiguration(configArray))
				{
					throw std::runtime_error("IMU: Could not configure MTmk4 pImp-> Aborting.");
				}
			}
			else
			{
				throw std::runtime_error("IMU: Unknown device while configuring. Aborting.");
			}
			
			// Put the device in measurement mode
			if (!pImp->gotoMeasurement())
			{
				throw std::runtime_error("IMU: Could not put device into measurement mode. Aborting.");
			}
		}
		void IMU::release()
		{
			pImp->close();
		}
		void IMU::updateData(ImuData &data)
		{
			XsByteArray imuData;
			XsMessageArray msgs;


			while (msgs.empty())
			{
				aris::core::msSleep(1);
				pImp->readDataToBuffer(imuData);
				pImp->processBufferedData(imuData, msgs);
			}

			//std::cout << "msg num:" << msgs.size()<<std::endl;

			for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
			{
				// Retrieve a packet
				XsDataPacket packet;
				if ((*it).getMessageId() == XMID_MtData2) 
				{
					packet.setMessage((*it));
					packet.setDeviceId(pImp->mtPort.deviceId());
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
				data.pmLhs = *pImp->pmImuGround2BodyGround;
				data.pmRhs = *pImp->pmBody2Imu;
			}

			msgs.clear();
		}
	}
}
