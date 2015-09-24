#include <Platform.h>

#include"Aris_IMU.h"

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
		class IMU::IMU_IMPLEMENT :public DeviceClass{};
		
		IMU::IMU()
		{
			pDevice = new IMU_IMPLEMENT;
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
				std::string portNumber = "/dev/ttyUSB0";
				int baudRate = 921600;

				XsPortInfo portInfo(portNumber, XsBaud::numericToRate(baudRate));
				portInfoArray.push_back(portInfo);
#endif
			}

			mtPort = portInfoArray.at(0);

			// Open the port with the detected device
			if (!pDevice->openPort(mtPort))
				throw std::runtime_error("IMU: could not open port.");

			// Put the device in configuration mode
			if (!pDevice->gotoConfig()) // Put the device into configuration mode before configuring the device
			{
				throw std::runtime_error("IMU: could not put device into configuration mode");
			}

			// Request the device Id to check the device type
			mtPort.setDeviceId(pDevice->getDeviceId());

			// Check if we have an MTi / MTx / MTmk4 device
			if (!mtPort.deviceId().isMt9c() && !mtPort.deviceId().isMtMk4())
			{
				throw std::runtime_error("IMU: No MTi / MTx / MTmk4 device found.");
			}

			// Check device type
			if (mtPort.deviceId().isMt9c())
			{
				XsOutputMode outputMode = XOM_Orientation; // output orientation data
				XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

																				  // set the device configuration
				if (!pDevice->setDeviceMode(outputMode, outputSettings))
				{
					throw std::runtime_error("IMU: Could not configure MT pDevice-> Aborting.");
				}
			}
			else if (mtPort.deviceId().isMtMk4())
			{
				XsOutputConfiguration quat(XDI_Quaternion, 100);
				XsOutputConfigurationArray configArray;
				configArray.push_back(quat);
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

			pDevice->readDataToBuffer(imuData);
			pDevice->processBufferedData(imuData, msgs);
			for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
			{
				// Retrieve a packet
				XsDataPacket packet;
				if ((*it).getMessageId() == XMID_MtData) {
					LegacyDataPacket lpacket(1, false);
					lpacket.setMessage((*it));
					lpacket.setXbusSystem(false, false);
					lpacket.setDeviceId(mtPort.deviceId(), 0);
					lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion, 0);	//lint !e534
					XsDataPacket_assignFromXsLegacyDataPacket(&packet, &lpacket, 0);
				}
				else if ((*it).getMessageId() == XMID_MtData2) {
					packet.setMessage((*it));
					packet.setDeviceId(mtPort.deviceId());
				}

				// Get the quaternion data
				XsQuaternion quaternion = packet.orientationQuaternion();

				// Convert packet to euler
				XsEuler euler = packet.orientationEuler();

				data.time = packet.timeOfArrival().msTimeOfDay();

				data.a = euler.pitch();
				data.b = euler.roll();
				data.c = euler.yaw();
			}
			msgs.clear();
			
			

		}
	}
}