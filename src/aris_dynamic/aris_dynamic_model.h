#ifndef ARIS_DYNAMIC_MODEL_
#define ARIS_DYNAMIC_MODEL_

#ifndef PI
#define PI 3.141592653589793
#endif

#include <aris_dynamic_model_base.h>


namespace Aris
{
	namespace Dynamic
	{
		class RevoluteJoint final :public JointBaseDim<5>
		{
		public:
			static std::string type() { return "revolute"; };
			virtual ~RevoluteJoint() = default;
			virtual std::string adamsType() const override { return std::move(type()); };

		private:
			explicit RevoluteJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit RevoluteJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void init() override;

			friend class ModelBase;
			friend class Model;
		};
		class TranslationalJoint final :public JointBaseDim<5>
		{
		public:
			static std::string type() { return "translational"; };
			virtual ~TranslationalJoint() = default;
			virtual std::string adamsType() const override { return std::move(type()); };

		private:
			explicit TranslationalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit TranslationalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void init() override;

			friend class ModelBase;
			friend class Model;
		};
		class UniversalJoint final :public JointBaseDim<4>
		{
		public:
			static std::string type() { return "universal"; };
			virtual ~UniversalJoint() = default;
			virtual std::string adamsType() const override { return std::move(type()); };
			virtual void update();

		private:
			explicit UniversalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit UniversalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void toAdamsCmd(std::ofstream &file) const;
			virtual void init() override;

			friend class ModelBase;
			friend class Model;
		};
		class SphericalJoint final :public JointBaseDim<3>
		{
		public:
			static std::string type() { return "spherical"; };
			virtual ~SphericalJoint() = default;
			virtual std::string adamsType() const override { return std::move(type()); };
			
		private:
			explicit SphericalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit SphericalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void init() override;

			friend class ModelBase;
			friend class Model;
		};

		class SingleComponentMotion final :public MotionBase
		{
		public:
			static std::string type() { return "single_component_motion"; };
			virtual ~SingleComponentMotion() = default;
			virtual std::string adamsType() const override { return std::move(type()); };
			virtual void update() override;

		private:
			explicit SingleComponentMotion(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ, int componentAxis);
			explicit SingleComponentMotion(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void init() override;
			void toAdamsCmd(std::ofstream &file) const override;

			int component_axis_;
		private:
			friend class ModelBase;
			friend class Model;
		};

		class SingleComponentForce final :public ForceBase
		{
		public:
			static std::string type() { return "single_component_force"; };
			virtual ~SingleComponentForce() = default;
			virtual std::string adamsType() const override { return std::move(type()); };
			virtual void update();

			void setComponentID(int id) { component_axis_ = id; };
			void setFce(double value) { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; };
			void setFce(double value, int componentID) { this->component_axis_ = componentID; setFce(value); };
			double fce()const { return fce_value_[component_axis_]; };

		private:
			explicit SingleComponentForce(ModelBase &model, const std::string &name, int id, Marker& makI, Marker& makJ, int componentID);
			explicit SingleComponentForce(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void toAdamsCmd(std::ofstream &file) const;

			int component_axis_;
			double fce_value_[6]{0};

			friend class ModelBase;
			friend class Model;
		};

		class Model :public ModelBase 
		{
		public:
			virtual void loadXml(const Aris::Core::XmlElement &ele)override;
			using ModelBase::loadXml;
		};
	}
}

#endif
