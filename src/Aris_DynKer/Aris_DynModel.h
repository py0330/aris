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
		class RevoluteJoint final :public JointBaseDim<5>
		{
		public:
			static std::string Type() { return "revolute"; };
			virtual ~RevoluteJoint() = default;
			virtual std::string AdamsType() const override { return std::move(Type()); };

		private:
			explicit RevoluteJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit RevoluteJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void Init() override;

			friend class ModelBase;
			friend class Model;
		};
		class TranslationalJoint final :public JointBaseDim<5>
		{
		public:
			static std::string Type() { return "translational"; };
			virtual ~TranslationalJoint() = default;
			virtual std::string AdamsType() const override { return std::move(Type()); };

		private:
			explicit TranslationalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit TranslationalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void Init() override;

			friend class ModelBase;
			friend class Model;
		};
		class UniversalJoint final :public JointBaseDim<4>
		{
		public:
			static std::string Type() { return "universal"; };
			virtual ~UniversalJoint() = default;
			virtual std::string AdamsType() const override { return std::move(Type()); };
			virtual void Update();

		private:
			explicit UniversalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit UniversalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void ToAdamsCmd(std::ofstream &file) const;
			virtual void Init() override;

			friend class ModelBase;
			friend class Model;
		};
		class SphericalJoint final :public JointBaseDim<3>
		{
		public:
			static std::string Type() { return "spherical"; };
			virtual ~SphericalJoint() = default;
			virtual std::string AdamsType() const override { return std::move(Type()); };
			
		private:
			explicit SphericalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit SphericalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void Init() override;

			friend class ModelBase;
			friend class Model;
		};

		class SingleComponentMotion final :public MotionBase
		{
		public:
			static std::string Type() { return "single_component_motion"; };
			virtual ~SingleComponentMotion() = default;
			virtual std::string AdamsType() const override { return std::move(Type()); };
			virtual void Update() override;

		private:
			explicit SingleComponentMotion(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ, int componentAxis);
			explicit SingleComponentMotion(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void Init() override;
			void ToAdamsCmd(std::ofstream &file) const override;

			int component_axis_;
		private:
			friend class ModelBase;
			friend class Model;
		};

		class SingleComponentForce final :public ForceBase
		{
		public:
			static std::string Type() { return "single_component_force"; };
			virtual ~SingleComponentForce() = default;
			virtual std::string AdamsType() const override { return std::move(Type()); };
			virtual void Update();

			void SetComponentID(int id) { componentID = id; };
			void SetFce(double value) { std::fill_n(fceI, 6, 0); fceI[componentID] = value; };
			void SetFce(double value, int componentID) { this->componentID = componentID; SetFce(value); };
			double GetFce()const { return fceI[componentID]; };

		private:
			explicit SingleComponentForce(ModelBase &model, const std::string &name, int id, Marker& makI, Marker& makJ, int componentID);
			explicit SingleComponentForce(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void ToAdamsCmd(std::ofstream &file) const;

			int componentID;
			double fceI[6];

			friend class ModelBase;
			friend class Model;
		};

		class Model :public ModelBase 
		{
		public:
			virtual void LoadXml(const Aris::Core::XmlElement &ele)override;
			using ModelBase::LoadXml;
		};
	}
}

#endif
