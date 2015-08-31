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
		class TRANSLATIONAL_JOINT final :public JOINT_BASE
		{
		public:
			virtual const char* GetType() const { return type; };
			virtual int GetCstDim() const { return DIMENSION; };

		private:
			static const char *const type;
			enum { DIMENSION = 5 };
			virtual void Initiate();

			TRANSLATIONAL_JOINT(MODEL *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ);

			double _PrtCstMtxI[6][DIMENSION];
			double _PrtCstMtxJ[6][DIMENSION];
			double _CstFce[DIMENSION];
			double _a_c[DIMENSION];

			friend class MODEL;
		};
		class UNIVERSAL_JOINT final :public JOINT_BASE
		{
		public:
			virtual const char* GetType() const { return type; };
			virtual int GetCstDim() const { return DIMENSION; };
			virtual void Update();

		private:
			static const char *const type;
			enum { DIMENSION = 4 };
			virtual void Initiate();
			UNIVERSAL_JOINT(MODEL *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ);
			double _PrtCstMtxI[6][DIMENSION];
			double _PrtCstMtxJ[6][DIMENSION];
			double _CstFce[DIMENSION];
			double _a_c[DIMENSION];

			friend class MODEL;
		};
		class SPHERICAL_JOINT final :public JOINT_BASE
		{
		public:
			virtual const char* GetType() const { return type; };
			virtual int GetCstDim() const { return DIMENSION; };

		private:
			static const char *const type;
			enum { DIMENSION = 3 };
			virtual void Initiate();
			SPHERICAL_JOINT(MODEL *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ);

			double _PrtCstMtxI[6][DIMENSION];
			double _PrtCstMtxJ[6][DIMENSION];
			double _CstFce[DIMENSION];
			double _a_c[DIMENSION];

			friend class MODEL;
		};

		class LINEAR_MOTION final :public MOTION_BASE
		{
		public:
			virtual ~LINEAR_MOTION() = default;
			virtual int GetCstDim() const { return 1; };
			virtual const char* GetType() const { return type; };
			virtual void Update();

		private:
			explicit LINEAR_MOTION(MODEL *pModel, const std::string &Name, int id, MOTION_MODE mode = POS_CONTROL, MARKER *pMakI = 0, MARKER *pMakJ = 0);

			virtual void _Initiate();
			//virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			//virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);

		private:
			static const char *const type;
			//MOTION_TYPE _Type;
			//MOTION_MODE _Mode;
			//MARKER *_pMakI, *_pMakJ;

			/*pos\vel\acc\fce of motion*/
			/*double _p_m[6];
			double _v_m[6];
			double _a_m[6];
			double _f_m[6];
			double _frc_coe[3];

			double _PrtCstMtxI[6][6];
			double _PrtCstMtxJ[6][6];
			double _a_c[6];

			/*for adams*/
			std::unique_ptr<AKIMA> posCurve;
			std::unique_ptr<AKIMA> fceCurve;

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
		
		private:
			static const char *const type;

			SINGLE_COMPONENT_FORCE(MODEL *pModel, const std::string &name, int id, MARKER* pMak, PART* pPrtNI, int componentID);
			SINGLE_COMPONENT_FORCE(MODEL *pModel, const std::string &name, int id, const Aris::Core::ELEMENT *xmlEle);

			int componentID;
			double fceI[6];
			MARKER *pMakI;

			friend class MODEL;
		};
	}
}

#endif
