#include <Platform.h>

#include <string>

#include "Aris_Core.h"
#include "Aris_IMU.h"
#include "Aris_DynKer.h"

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "deviceclass.h"

XsPortInfo mtPort;

namespace Aris
{
	namespace Sensor
	{
		void IMU_DATA::ToBodyPm(double *pm) const
		{
			ToBodyPm(pm, this->yaw);
		}
		void IMU_DATA::ToBodyPm(double *pm, double yawValue) const
		{
			double tem_pm[16];
			double pe[6]{ 0,0,0,yawValue, pitch, roll };
			Aris::DynKer::s_pe2pm(pe, tem_pm, "321");
			Aris::DynKer::s_pm_dot_pm(pmLhs, tem_pm, pmRhs, pm);
		}
		void IMU_DATA::ToBodyEul(double *eul, const char *eulType) const
		{
			this->ToBodyEul(eul, this->yaw, eulType);
		}
		void IMU_DATA::ToBodyEul(double *eul, double yawValue, const char *eulType) const
		{
			double pm[16], pe[6];
			this->ToBodyPm(pm, yawValue);
			Aris::DynKer::s_pm2pe(pm, pe, eulType);
			std::copy_n(pe + 3, 3, eul);
		}
		void IMU_DATA::ToBodyOmega(double *omega) const
		{
			
		}
		void IMU_DATA::ToBodyOmega(double *omega, double yawValue) const
		{
			double pm[16];
			ToBodyPm(pm, yawValue);


		}
		void IMU_DATA::ToBodyAcc(double *acc) const
		{
		}
		void IMU_DATA::ToBodyAcc(double *acc, double yawValue) const
		{
		}
		
		
		
		class IMU::IMU_IMP :public DeviceClass
		{
		public:
			std::string port;
			int baudRate;
			int sampleRate;

			double pmImuGround2BodyGround[4][4];
			double pmBody2Imu[4][4];
		};
		
		IMU::IMU() 
		{
			pDevice = new IMU_IMP;
			pDevice->port = "/dev/ttyUSB0";
			pDevice->baudRate = 921600;
			pDevice->sampleRate = 400;

			std::fill_n(&pDevice->pmImuGround2BodyGround[0][0], 16, 0);
			pDevice->pmImuGround2BodyGround[0][0] = -1;
			pDevice->pmImuGround2BodyGround[1][2] = 1;
			pDevice->pmImuGround2BodyGround[2][1] = 1;
			pDevice->pmImuGround2BodyGround[3][3] = 1;

			std::fill_n(&pDevice->pmBody2Imu[0][0], 16, 0);
			pDevice->pmBody2Imu[0][0] = 1;
			pDevice->pmBody2Imu[1][2] = 1;
			pDevice->pmBody2Imu[2][1] = -1;
			pDevice->pmBody2Imu[3][3] = 1;
		}
		IMU::IMU(const Aris::Core::ELEMENT *xmlEle)
		{
			pDevice = new IMU_IMP;
			
#ifdef PLATFORM_IS_LINUX
			pDevice->port = xmlEle->Attribute("portLinux");
#endif
			pDevice->baudRate = std::stoi(xmlEle->Attribute("baudRate"));
			pDevice->sampleRate = std::stoi(xmlEle->Attribute("sampleRate"));;

			Aris::DynKer::CALCULATOR c;
			c.AddVariable("PI", PI);
			
			auto m = c.CalculateExpression(xmlEle->Attribute("PeImuGround2BodyGound"));
			Aris::DynKer::s_pe2pm(m.Data(), &pDevice->pmImuGround2BodyGround[0][0]);
			m = c.CalculateExpression(xmlEle->Attribute("PeImu2Body"));
			double pmImu2Body[16];
			Aris::DynKer::s_pe2pm(m.Data(), pmImu2Body);
			Aris::DynKer::s_inv_pm(pmImu2Body, &pDevice->pmBody2Imu[0][0]);

		}
		IMU::~IMU()
		{
			delete pDevice;
		}

		void IMU::Initiate()
		{
			XsPortInfoArray portInfoArray;
			xsEnumerateUsbDevices(portInfoArray);

			if (!portInfoArray.size())
			{
#ifdef PLATFORM_IS_WINDOWS
				throw std::runtime_error("IMU: failed to find IMU sensor");
#endif
#ifdef PLATFORM_IS_LINUX
				XsPortInfo portInfo(pDevice->port, XsBaud::numericToRate(pDevice->baudRate));
				portInfoArray.push_back(portInfo);
#endif
			}
			
			mtPort = portInfoArray.at(0);
			
			// Open the port with the detected device
			if (!pDevice->openPort(mtPort))
				throw std::runtime_error("IMU: could not open port.");
			
			Aris::Core::Sleep(10);

			// Put the device in configuration mode
			if (!pDevice->gotoConfig()) // Put the device into configuration mode before configuring the device
			{
				throw std::runtime_error("IMU: could not put device into configuration mode");
			}
			
			// Request the device Id to check the device type
			mtPort.setDeviceId(pDevice->getDeviceId());
			
			// Check if we have an MTi / MTx / MTmk4 device
			if (!mtPort.deviceId().isMtMk4())
			{
				throw std::runtime_error("IMU: No MTi / MTx / MTmk4 device found.");
			}
			
			// Check device type
			if (mtPort.deviceId().isMtMk4())
			{
				XsOutputConfiguration config0(XDI_Quaternion, pDevice->sampleRate);
				XsOutputConfiguration config1(XDI_DeltaQ, pDevice->sampleRate);
				XsOutputConfiguration config2(XDI_DeltaV, pDevice->sampleRate);
				XsOutputConfiguration config3(XDI_Acceleration, pDevice->sampleRate);

				XsOutputConfigurationArray configArray;
				configArray.push_back(config0);
				configArray.push_back(config1);
				configArray.push_back(config2);
				configArray.push_back(config3);
				if (!pDevice->setOutputConfiguration(configArray))
				{
					throw std::runtime_error("IMU: Could not configure MTmk4 pDevice-> Aborting.");
				}
			}
			else
			{
				throw std::runtime_error("IMU: Unknown device while configuring. Aborting.");
			}
			
			// Put the device in measurement mode
			if (!pDevice->gotoMeasurement())
			{
				throw std::runtime_error("IMU: Could not put device into measurement mode. Aborting.");
			}
		}
		void IMU::Release()
		{
			pDevice->close();
		}
		void IMU::UpdateData(IMU_DATA &data)
		{
			XsByteArray imuData;
			XsMessageArray msgs;


			while (msgs.empty())
			{
				Aris::Core::Sleep(1);
				pDevice->readDataToBuffer(imuData);
				pDevice->processBufferedData(imuData, msgs);
			}

			std::cout << "msg num:" << msgs.size()<<std::endl;

			for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
			{
				// Retrieve a packet
				XsDataPacket packet;
				if ((*it).getMessageId() == XMID_MtData2) 
				{
					packet.setMessage((*it));
					packet.setDeviceId(mtPort.deviceId());
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
				data.pmLhs = *pDevice->pmImuGround2BodyGround;
				data.pmRhs = *pDevice->pmBody2Imu;
			}

			msgs.clear();
		}
	}
}