#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>

using namespace std;


#include <Aris_DynModel.h>

#ifdef PLATFORM_IS_WINDOWS
#include <stdlib.h>
#endif
#ifdef PLATFORM_IS_LINUX
#include <stdlib.h>
#endif

extern "C"
{
#include <cblas.h>
}
#include <lapacke.h>

namespace Aris
{
	namespace DynKer
	{
		const char *const TRANSLATIONAL_JOINT::type = "translational";
		TRANSLATIONAL_JOINT::TRANSLATIONAL_JOINT(MODEL *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: JOINT_BASE(pModel, Name, id, pMakI, pMakJ, *_PrtCstMtxI, *_PrtCstMtxJ, _CstFce, _a_c)
		{
		}
		void TRANSLATIONAL_JOINT::Initiate()
		{
			JOINT_BASE::Initiate();

			double loc_cst[6][DIMENSION];
			std::memset(*loc_cst, 0, sizeof(loc_cst));

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[3][2] = 1;
			loc_cst[4][3] = 1;
			loc_cst[5][4] = 1;

			s_dgemm(6, 5, 6, 1, *_tm_I2M, 6, *loc_cst, DIMENSION, 0, *_PrtCstMtxI, DIMENSION);
		};
		void TRANSLATIONAL_JOINT::Update()
		{
			double _pm_M2N[4][4];
			double _tm_M2N[6][6];
			double _tem_v1[6], _tem_v2[6];

			/* Get pm M2N */
			s_pm_dot_pm(GetMakJ()->GetFatherPrt()->GetPrtPmPtr(), GetMakI()->GetFatherPrt()->GetPmPtr(), *_pm_M2N);
			s_tmf(*_pm_M2N, *_tm_M2N);

			/*update PrtCstMtx*/
			//s_dgemm(6, 5, 6, -1, *_tm_M2N, 6, *_PrtCstMtxI, DIMENSION, 0, *_PrtCstMtxJ, DIMENSION);
			s_tf_n(DIMENSION, -1, *_pm_M2N, *_PrtCstMtxI, 0, *_PrtCstMtxJ);
			/*update A_c*/
			s_dgemmTN(6, 1, 6, -1, *_tm_M2N, 6, GetMakJ()->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1, 1);
			s_cv(GetMakI()->GetFatherPrt()->GetPrtVelPtr(), _tem_v1, _tem_v2);
			s_dgemmTN(5, 1, 6, 1, *_PrtCstMtxI, DIMENSION, _tem_v2, 1, 0, _a_c, 1);
		};
		
		const char *const UNIVERSAL_JOINT::type = "universal";
		UNIVERSAL_JOINT::UNIVERSAL_JOINT(MODEL *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: JOINT_BASE(pModel, Name, id, pMakI, pMakJ, *_PrtCstMtxI, *_PrtCstMtxJ, _CstFce, _a_c)
		{
		}
		void UNIVERSAL_JOINT::Initiate()
		{
			JOINT_BASE::Initiate();
			
			double loc_cst[6][DIMENSION];
			std::memset(*loc_cst, 0, sizeof(loc_cst));

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_dgemm(6, 3, 6, 1, *_tm_I2M, 6, *loc_cst, DIMENSION, 0, *_PrtCstMtxI, DIMENSION);
		};
		void UNIVERSAL_JOINT::Update()
		{
			double _pm_M2N[4][4];
			double _tm_M2N[6][6];
			double _tem_v1[6], _tem_v2[6];

			/* Get pm M2N */
			s_pm_dot_pm(GetMakJ()->GetFatherPrt()->GetPrtPmPtr(), GetMakI()->GetFatherPrt()->GetPmPtr(), *_pm_M2N);
			s_tmf(*_pm_M2N, *_tm_M2N);

			double v[3];
			double a, a_dot;

			std::fill_n(_a_c, UNIVERSAL_JOINT::GetCstDim(), 0);

			/*update PrtCstMtx*/
			//get sin(a) and cos(a)
			GetMakI()->Update();
			GetMakJ()->Update();

			double s = GetMakI()->GetPmPtr()[2] * GetMakJ()->GetPmPtr()[1]
				+ GetMakI()->GetPmPtr()[6] * GetMakJ()->GetPmPtr()[5]
				+ GetMakI()->GetPmPtr()[10] * GetMakJ()->GetPmPtr()[9];

			double c = GetMakI()->GetPmPtr()[1] * GetMakJ()->GetPmPtr()[1]
				+ GetMakI()->GetPmPtr()[5] * GetMakJ()->GetPmPtr()[5]
				+ GetMakI()->GetPmPtr()[9] * GetMakJ()->GetPmPtr()[9];

			a = std::atan2(s, c);

					/*edit CstMtxI*/
			_PrtCstMtxI[3][3] = -_tm_I2M[3][4] * s + _tm_I2M[3][5] * c;
			_PrtCstMtxI[4][3] = -_tm_I2M[4][4] * s + _tm_I2M[4][5] * c;
			_PrtCstMtxI[5][3] = -_tm_I2M[5][4] * s + _tm_I2M[5][5] * c;
			
			s_tf_n(DIMENSION, -1, *_pm_M2N, *_PrtCstMtxI, 0, *_PrtCstMtxJ);
			//s_dgemm(6, 4, 6, -1, *_tm_M2N, 6, *_PrtCstMtxI, DIMENSION, 0, *_PrtCstMtxJ, DIMENSION);
					/*update A_c*/
					  /*calculate a_dot*/
			v[0] = GetMakJ()->GetVelPtr()[3] - GetMakI()->GetVelPtr()[3];
			v[1] = GetMakJ()->GetVelPtr()[4] - GetMakI()->GetVelPtr()[4];
			v[2] = GetMakJ()->GetVelPtr()[5] - GetMakI()->GetVelPtr()[5];

			a_dot = GetMakI()->GetPmPtr()[0] * v[0] + GetMakI()->GetPmPtr()[4] * v[1] + GetMakI()->GetPmPtr()[8] * v[2];
			/*calculate part m*/
			v[0] = -c*a_dot;
			v[1] = -s*a_dot;

			s_dgemmTN(2, 1, 6, 1, *_tm_I2M + 24, 6, GetMakI()->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1 + 4, 1);
			_a_c[3] -= v[0] * _tem_v1[4] + v[1] * _tem_v1[5];
			/*calculate part n*/
			s_dgemmTN(6, 1, 6, 1, *_tm_M2N, 6, GetMakJ()->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1, 1);
			s_cv(-1, GetMakI()->GetFatherPrt()->GetPrtVelPtr(), _tem_v1, 0, _tem_v2);
			s_dgemmTN(4, 1, 6, 1, *_PrtCstMtxI, DIMENSION, _tem_v2, 1, 1, &_a_c[0], 1);
			s_dgemmTN(6, 1, 6, 1, *_tm_I2M, 6, _tem_v1, 1, 0, _tem_v2, 1);
			_a_c[3] += v[0] * _tem_v2[4] + v[1] * _tem_v2[5];
		};

		const char *const SPHERICAL_JOINT::type = "spherical";
		SPHERICAL_JOINT::SPHERICAL_JOINT(MODEL *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: JOINT_BASE(pModel, Name, id, pMakI, pMakJ, *_PrtCstMtxI, *_PrtCstMtxJ, _CstFce, _a_c)
		{
		}
		void SPHERICAL_JOINT::Initiate()
		{
			JOINT_BASE::Initiate();
			
			double loc_cst[6][DIMENSION];
			memset(*loc_cst, 0, sizeof(loc_cst));

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_dgemm(6, 3, 6, 1, *_tm_I2M, 6, *loc_cst, DIMENSION, 0, *_PrtCstMtxI, DIMENSION);
		};
		void SPHERICAL_JOINT::Update()
		{
			double _pm_M2N[4][4];
			double _tm_M2N[6][6];
			double _tem_v1[6], _tem_v2[6];

			/* Get pm M2N */
			s_pm_dot_pm(GetMakJ()->GetFatherPrt()->GetPrtPmPtr(), GetMakI()->GetFatherPrt()->GetPmPtr(), *_pm_M2N);
			s_tmf(*_pm_M2N, *_tm_M2N);

			/*update PrtCstMtx*/
			//s_dgemm(6, 3, 6, -1, *_tm_M2N, 6, *_PrtCstMtxI, DIMENSION, 0, *_PrtCstMtxJ, DIMENSION);
			s_tf_n(DIMENSION, -1, *_pm_M2N, *_PrtCstMtxI, 0, *_PrtCstMtxJ);
			/*update A_c*/
			s_dgemmTN(6, 1, 6, -1, *_tm_M2N, 6, GetMakJ()->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1, 1);
			s_cv(GetMakI()->GetFatherPrt()->GetPrtVelPtr(), _tem_v1, _tem_v2);
			s_dgemmTN(3, 1, 6, 1, *_PrtCstMtxI, DIMENSION, _tem_v2, 1, 0, &_a_c[0], 1);
		};

		const char *const LINEAR_MOTION::type = "linear";
		LINEAR_MOTION::LINEAR_MOTION(MODEL *pModel, const std::string &Name, int id, MOTION_BASE::MOTION_MODE mode, MARKER *pMakI, MARKER *pMakJ)
			: MOTION_BASE(pModel, Name, id, mode, pMakI, pMakJ)
		{
		}
		void LINEAR_MOTION::_Initiate()
		{
			double _tmf_I2M[6][6];
			double loc_cst[6][6];

			memset(*_PrtCstMtxI, 0, sizeof(_PrtCstMtxI));
			memset(*_PrtCstMtxJ, 0, sizeof(_PrtCstMtxJ));

			memset(*loc_cst, 0, sizeof(loc_cst));

			/* Get tm I2M */
			s_tmf(_pMakI->GetPrtPmPtr(), *_tmf_I2M);
			loc_cst[2][0] = 1;
			s_dgemm(6, 1, 6, 1, *_tmf_I2M, 6, *loc_cst, 6, 0, *_PrtCstMtxI, 6);
		}
		void LINEAR_MOTION::Update()
		{
			double _pm_M2N[4][4];
			double _tmf_M2N[6][6];

			double tem_v1[6], tem_v2[6];

			memset(*_PrtCstMtxJ, 0, sizeof(double) * 36);
			memset(_a_c, 0, sizeof(double) * 6);

			/* Get tmf M2N */
			s_pm_dot_pm(_pMakJ->GetFatherPrt()->GetPrtPmPtr(), _pMakI->GetFatherPrt()->GetPmPtr(), *_pm_M2N);
			s_tmf(*_pm_M2N, *_tmf_M2N);
			s_dgemm(6, 1, 6, -1, *_tmf_M2N, 6, *_PrtCstMtxI, 6, 0, *_PrtCstMtxJ, 6);

			switch (_Mode)
			{
			case POS_CONTROL:
				s_dgemmTN(6, 1, 6, -1, *_tmf_M2N, 6, _pMakJ->GetFatherPrt()->GetPrtVelPtr(), 1, 0, tem_v1, 1);
				s_cv(_pMakI->GetFatherPrt()->GetPrtVelPtr(), tem_v1, tem_v2);
				s_dgemmTN(1, 1, 6, 1, *_PrtCstMtxI, 6, tem_v2, 1, 0, &_a_c[0], 1);

				_a_c[0] += _a_m[0];
				break;
			case FCE_CONTROL:
				break;
			}
		}

		void SINGLE_COMPONENT_FORCE::Update()
		{
			s_tf(this->pMakI->GetPrtPmPtr(), fceI, _PrtFceI);
			double pm[16];
			s_inv_pm_dot_pm(_pPrtN->GetPmPtr(), pMakI->GetPmPtr(), pm);
			s_tf(-1, pm, fceI, 0, _PrtFceJ);
		}
		SINGLE_COMPONENT_FORCE::SINGLE_COMPONENT_FORCE(MODEL *pModel, const std::string &name, int id, MARKER* makI, PART* pPrtN, int componentID)
			: FORCE_BASE(pModel, name, id, makI->GetFatherPrt(), pPrtN)
			, pMakI(makI)
			, componentID(componentID)
		{

		}

	}
}
