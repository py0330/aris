/*!
* \file Aris_DynKer.h
* \brief 概述
* \author 潘阳
* \version 1.0.0.0
* \date 2014/4/5
*/

#ifndef Aris_DynModel_H
#define Aris_DynModel_H

#ifndef PI
#define PI 3.141592653589793
#endif

#include <Aris_DynModelBase.h>

namespace Aris
{
	namespace DynKer
	{
		class TranslationalJoint final :public JointBaseDim<5>
		{
		public:
			virtual ~TranslationalJoint() = default;
			virtual const char* GetType() const { return type; };

		private:
			static const char *const type;
			explicit TranslationalJoint(ModelBase *pModel, const std::string &Name, int id, Marker *pMakI, Marker *pMakJ);
			explicit TranslationalJoint(ModelBase *pModel, const std::string &Name, int id, const Aris::Core::XmlElement *ele);
			virtual void Init();

			friend class ModelBase;
			friend class Model;
		};
		class UniversalJoint final :public JointBaseDim<4>
		{
		public:
			virtual ~UniversalJoint() = default;
			virtual const char* GetType() const { return type; };
			virtual void Update();

		private:
			static const char *const type;
			explicit UniversalJoint(ModelBase *pModel, const std::string &Name, int id, Marker *pMakI, Marker *pMakJ);
			explicit UniversalJoint(ModelBase *pModel, const std::string &Name, int id, const Aris::Core::XmlElement *ele);
			virtual void ToAdamsCmd(std::ofstream &file) const;
			virtual void Init();

			friend class ModelBase;
			friend class Model;
		};
		class SphericalJoint final :public JointBaseDim<3>
		{
		public:
			virtual ~SphericalJoint() = default;
			virtual const char* GetType() const { return type; };

		private:
			static const char *const type;
			explicit SphericalJoint(ModelBase *pModel, const std::string &Name, int id, Marker *pMakI, Marker *pMakJ);
			explicit SphericalJoint(ModelBase *pModel, const std::string &Name, int id, const Aris::Core::XmlElement *ele);
			virtual void Init();

			friend class ModelBase;
			friend class Model;
		};

		class LinearMotion final :public MotionBase
		{
		public:
			virtual ~LinearMotion() = default;
			virtual const char* GetType() const { return type; };
			virtual void Update();

		private:
			static const char *const type;
			explicit LinearMotion(ModelBase *pModel, const std::string &Name, int id, Marker *pMakI, Marker *pMakJ);
			explicit LinearMotion(ModelBase *pModel, const std::string &Name, int id, const Aris::Core::XmlElement *pEle);
			virtual void Init();

		private:

			friend class ModelBase;
			friend class Model;
		};

		class SingleComponentForce final :public ForceBase
		{
		public:
			virtual ~SingleComponentForce() = default;
			virtual void Update();
			virtual const char* GetType() const { return type; };

			void SetComponentID(int id) { componentID = id; };
			void SetFce(double value) { std::fill_n(fceI, 6, 0); fceI[componentID] = value; };
			void SetFce(double value, int componentID) { this->componentID = componentID; SetFce(value); };
			double GetFce()const { return fceI[componentID]; };

		private:
			static const char *const type;
			explicit SingleComponentForce(ModelBase *pModel, const std::string &name, int id, Marker* pMakI, Marker* pMakJ, int componentID);
			explicit SingleComponentForce(ModelBase *pModel, const std::string &name, int id, const Aris::Core::XmlElement *xmlEle);
			virtual void ToAdamsCmd(std::ofstream &file) const;

			int componentID;
			double fceI[6];

			friend class ModelBase;
			friend class Model;
		};

		class Model :public ModelBase 
		{
		public:
			virtual void LoadXml(const Aris::Core::XmlElement *pEle);
			virtual void LoadXml(const Aris::Core::XmlDocument &doc) { this->ModelBase::LoadXml(doc); };
			virtual void LoadXml(const char *filename) { this->ModelBase::LoadXml(filename); };
		};
	}
}

#endif
