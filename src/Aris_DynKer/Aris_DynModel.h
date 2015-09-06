/*!
* \file Aris_DynKer.h
* \brief 概述
* \author 潘阳
* \version 1.0.0.0
* \date 2014/4/5
*/

#ifndef Aris_DynModel_H
#define Aris_DynModel_H

#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#ifndef _SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif
#endif

#ifndef PI
#define PI 3.141592653589793
#endif

#include <Aris_DynModelBase.h>


namespace Aris
{
	namespace DynKer
	{
		class TRANSLATIONAL_JOINT final :public JOINT_BASE_DIM<5>
		{
		public:
			virtual ~TRANSLATIONAL_JOINT() = default;
			virtual const char* GetType() const { return type; };

		private:
			static const char *const type;
			explicit TRANSLATIONAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ);
			explicit TRANSLATIONAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele);
			virtual void Initiate();

			friend class MODEL_BASE;
			friend class MODEL;
		};
		class UNIVERSAL_JOINT final :public JOINT_BASE_DIM<4>
		{
		public:
			virtual ~UNIVERSAL_JOINT() = default;
			virtual const char* GetType() const { return type; };
			virtual void Update();

		private:
			static const char *const type;
			UNIVERSAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ);
			UNIVERSAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele);
			virtual void ToAdamsCmd(std::ofstream &file) const;
			virtual void Initiate();

			friend class MODEL_BASE;
			friend class MODEL;
		};
		class SPHERICAL_JOINT final :public JOINT_BASE_DIM<3>
		{
		public:
			virtual ~SPHERICAL_JOINT() = default;
			virtual const char* GetType() const { return type; };

		private:
			static const char *const type;
			SPHERICAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ);
			SPHERICAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele);
			virtual void Initiate();

			friend class MODEL_BASE;
			friend class MODEL;
		};

		class LINEAR_MOTION final :public MOTION_BASE
		{
		public:
			virtual ~LINEAR_MOTION() = default;
			virtual const char* GetType() const { return type; };
			virtual void Update();

		private:
			static const char *const type;
			explicit LINEAR_MOTION(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI = 0, MARKER *pMakJ = 0);
			explicit LINEAR_MOTION(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *pEle);
			virtual void Initiate();

		private:
			/*for adams*/
			std::unique_ptr<AKIMA> posCurve;
			std::unique_ptr<AKIMA> fceCurve;

			friend class MODEL_BASE;
			friend class MODEL;
		};

		class SINGLE_COMPONENT_FORCE final :public FORCE_BASE
		{
		public:
			virtual ~SINGLE_COMPONENT_FORCE() = default;
			virtual void Update();
			virtual const char* GetType() const { return type; };

			void SetComponentID(int id) { componentID = id; };
			void SetFce(double value) { std::fill_n(fceI, 6, 0); fceI[componentID] = value; };
			void SetFce(double value, int componentID) { this->componentID = componentID; SetFce(value); };
			double GetFce()const { return fceI[componentID]; };

		private:
			static const char *const type;
			SINGLE_COMPONENT_FORCE(MODEL_BASE *pModel, const std::string &name, int id, MARKER* pMakI, MARKER* pMakJ, int componentID);
			SINGLE_COMPONENT_FORCE(MODEL_BASE *pModel, const std::string &name, int id, const Aris::Core::ELEMENT *xmlEle);
			virtual void ToAdamsCmd(std::ofstream &file) const;

			int componentID;
			double fceI[6];
			MARKER *pMakI;
			MARKER *pMakJ;

			friend class MODEL_BASE;
			friend class MODEL;
		};

		class MODEL :public MODEL_BASE 
		{
		public:
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);
		};
	}
}

#endif
