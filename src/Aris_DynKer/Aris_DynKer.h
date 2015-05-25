/*!
* \file Aris_DynKer.h
* \brief 概述
* \author 潘阳
* \version 1.0.0.0
* \date 2014/4/5
*/

#ifndef Aris_DynKer_H
#define Aris_DynKer_H

#ifndef PI
#define PI 3.141592653589793
#endif

#include "Aris_XML.h"
#include "Aris_ExpCal.h"
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <functional>

namespace Aris
{
	namespace DynKer
	{
		void s_dgeev(int n, double* a, int lda, double* wr, double* wi);
		
		void dsp(const double *p, const int m, const int n, const int begin_row = 0, const int begin_col = 0, int ld = 0);
		void dlmwrite(const char *FileName, const double *pMatrix, const unsigned int m, const unsigned int n);
		void dlmread(const char *FileName, double *pMatrix);
		
		int s_sgn(double x);

		void s_cro3(const double *cro_vec_in, const double *vec_in, double *vec_out);
		void s_cm3(const double *cro_vec_in, double *cm_out);

		void s_inv_pm(const double *pm_in, double *pm_out);
		void s_inv_im(const double *im_in, double *im_out);
		void s_axes2pm(const double *origin, const double *firstAxisPnt, const double *secondAxisPnt, double *pm_out, const char *axesOrder = "xy");
		void s_pm2ep(const double *pm_in, double *ep_out, const char *EurType="313");
		void s_ep2pm(const double *ep_in, double *pm_out, const char *EurType="313");
		void s_tmf(const double *pm_in, double *tm_out);
		void s_tmv(const double *pm_in, double *tmd_out);
		void s_cmf(const double *vel_in, double *cm_out);
		void s_cmv(const double *vel_in, double *cmd_out);
		void s_i2i(const double *from_pm_in, const double *from_im_in, double *to_im_out);
		/** \brief 计算点加速度
		*
		* 用来将6维空间速度从一个坐标系中转化到另一个坐标系中。例如将B坐标系中的速度转换到A坐标系中。
		*
		* \param relative_pm_in 表示两个坐标系之间的位姿矩阵。即B相对于A的位姿矩阵。
		* \param relative_vel_in 表示两个坐标系之间的相对速度。即B相对于A的空间速度向量，这个向量在坐标系A中表达。
		* \param from_vel_in 表示坐标系B中待转换的速度向量。
		* \param to_vel_out 表示转换完成的速度向量，即A坐标系中的速度向量。
		*/
		void s_v2v(const double *relative_pm_in, const double *relative_vel_in, const double *from_vel_in, double *to_vel_out);
		void s_a2a(const double *relative_pm_in, const double *relative_vel_in, const double *relative_acc_in,
			const double *from_vel_in, const double *from_acc_in, double *to_acc_out, double *to_vel_out = 0);
		void s_pnt2pnt(const double *relative_pm_in, const double *from_pnt, double *to_pnt_out);
		/** \brief 计算点加速度
		*
		* 转化点速度的坐标系。
		*
		*/
		void s_pv2pv(const double *relative_pm_in, const double *relative_vel_in, 
			const double *from_pnt, const double *from_pv, double *to_pv_out, double *to_pnt_out = 0);
		void s_pa2pa(const double *relative_pm_in, const double *relative_vel_in, const double *inv_relative_acc_in,
			const double *from_pnt, const double *from_pv, const double *from_pa,
			double *to_pa_out, double *to_pv_out = 0, double *to_pnt_out = 0);
		void s_inv_pv2pv(const double *inv_relative_pm_in, const double *inv_relative_vel_in,
			const double *from_pnt, const double *from_pv, double *to_pv_out, double *to_pnt_out = 0);
		void s_inv_pa2pa(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *inv_relative_acc_in,
			const double *from_pnt, const double *from_pv, const double *from_pa, 
			double *to_pa_out, double *to_pv_out = 0, double *to_pnt_out = 0);
		void s_mass2im(const double mass_in, const double * inertia_in, const double *pm_in, double *im_out);
		void s_gamma2im(const double * gamma_in, double *im_out);
		void s_im2gamma(const double * im_in, double *gamma_out);

		void s_pv(const double *pnt_in, const double *vel_in, double *pv_out);
		/** \brief 计算点加速度
		*
		* 根据数据点位置、空间速度和空间加速度来计算该点的加速度。
		*
		*/
		void s_pa(const double *pnt_in, const double *vel_in, const double *acc_in, double *pnt_acc_out);

		void s_block_cpy(const int &block_size_m, const int &block_size_n,
			const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld);

		void s_dlt_col(const int &dlt_col_num, const int *col_index, const int &m, const int &n, double *A, const int &ldA);

		void s_pm_dot_pm(const double *pm1_in, const double *pm2_in, double *pm_out);
		void s_inv_pm_dot_pm(const double *inv_pm1_in, const double *pm2_in, double *pm_out);
		void s_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out);
		void s_inv_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out);
		void s_tmf_dot_fce(const double *tm_in, const double *fce_in, double *fce_out);
		void s_tmv_dot_vel(const double *tmd_in, const double *vel_in, double *vel_out);
		void s_im_dot_gravity(const double *im_in, const double *gravity, double *gravity_fce_out);

		void s_dscal(const int n, const double a, double *x, const int incx);
		double s_dnrm2(const int n, const double *x, const int incx);
		void s_daxpy(const int N, const double alpha, const double *X, const int incX, double *Y, const int incY);
		void s_swap(const int N, double *X, const int incX, double *Y, const int incY);
		void s_transpose(const int m, const int n, const double *A, const int ldA, double *B_out, const int ldB);

		void s_dgeinv(const int n, double* A,const int lda, int *ipiv);
		void s_dgemm(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc);
		void s_dgemmTN(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc);
		void s_dgemmNT(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc);
		void s_dgesv(int n, int nrhs, double* a, int lda, int* ipiv, double* b, int ldb);
		void s_dgesvT(int n, int nrhs, double* a, int lda, int* ipiv, double* b, int ldb);
		void s_dgelsd(int m, int n, int nrhs, double* a, int lda, double* b, int ldb, double* s, double rcond, int* rank);
		void s_dgelsdT(int m, int n, int nrhs, double* a, int lda, double* b, int ldb, double* s, double rcond, int* rank);

		void s_akima(unsigned inNum, unsigned outNum, const double *x_in, const double *y_in, const double *x, double *y,unsigned order=0);

		class PART;
		class MARKER;
		class JOINT;
		class MOTION;
		class FORCE;

		class ENVIRONMENT;
		class MODEL;

		class OBJECT
		{
		protected:
			std::string _Name;
			bool _IsActive;

			MODEL *_pModel;

		public:
			OBJECT(MODEL *pModel, const std::string &);
			virtual ~OBJECT();

			std::string GetName() const;
			bool GetActive() const;
			int Activate();
			int Deactivate();
		};
		/** \brief 部件类型
		*
		* 定义模型中每个部件的数据结构
		* 
		*/
		class PART:public OBJECT
		{
			friend class MODEL;

		private:
			std::map<std::string, unsigned> _markerNames;
		
		private:
			double _Pm[4][4];
			double _Vel[6];
			double _Acc[6];

		private:
			double _PrtIm[6][6];
			double _PrtPm[4][4];//inverse of the _Pm
			double _PrtTmf[6][6];//inverse of the _Tmf
			double _PrtTmv[6][6];//inverse of the _Tmv
			double _PrtCmf[6][6];
			double _PrtCmv[6][6];
			double _PrtGravity[6];
			double _PrtAcc[6];
			double _PrtVel[6];
			double _PrtFg[6];
			double _PrtFv[6];

			int _RowId;

		private:
			PART(MODEL *pModel
				, const std::string &Name = ""
				, const double *PrtIm = nullptr
				, const double *pm = nullptr
				, const double *Vel = nullptr
				, const double *Acc = nullptr);
		
			PART(const PART&) = delete;
			PART(PART&&) = delete;
			PART & operator =(const PART &) = delete;
			PART & operator =(PART &&) = delete;
		public:
			~PART() = default;

		public:
			double* GetPmPtr() {return *_Pm; };
			double* GetVelPtr() { return _Vel; };
			double* GetAccPtr() { return _Acc; };

			const double* GetPmPtr() const{ return *_Pm; };
			const double* GetVelPtr() const{ return _Vel; };
			const double* GetAccPtr() const{ return _Acc; };

			double* GetPrtPmPtr() { return *_PrtPm; };
			double* GetPrtVelPtr() { return _PrtVel; };
			double* GetPrtAccPtr() { return _PrtAcc; };

			const double* GetPrtImPtr() const{ return *_PrtIm; };
			const double* GetPrtPmPtr() const{ return *_PrtPm; };
			const double* GetPrtTmfPtr() const{ return *_PrtTmf; };
			const double* GetPrtTmvPtr() const{ return *_PrtTmv; };
			const double* GetPrtCmfPtr() const{ return *_PrtCmf; };
			const double* GetPrtCmvPtr() const{ return *_PrtCmv; };
			const double* GetPrtAccPtr() const{ return _PrtAcc; };
			const double* GetPrtVelPtr() const{ return _PrtVel; };
			const double* GetPrtFgPtr() const{ return _PrtFg; };
			const double* GetPrtFvPtr() const{ return _PrtFv; };
			const double* GetPrtGravityPtr() const{ return _PrtGravity; };

			void UpdateInPrt();
		
		public:
			void SetPrtIm(const double *pPrtIm){ memcpy(_PrtIm, pPrtIm, sizeof(_PrtIm)); };
			void SetPm(const double *pPm){ memcpy(_Pm, pPm, sizeof(_Pm)); };
			void SetVel(const double *pVel){ memcpy(_Vel, pVel, sizeof(_Vel)); };
			void SetAcc(const double *pAcc){ memcpy(_Acc, pAcc, sizeof(_Acc)); };

		public:
			unsigned GetID() const;
			MARKER* GetMarker(const std::string &Name);
			const MARKER* GetMarker(const std::string &Name)const;
			MARKER* AddMarker(const std::string &Name, const double *pm = 0, MARKER *pRelativeTo = 0);

		public:
			std::string graphicFilePath;

		public:
			void ToXMLElement(Aris::Core::ELEMENT *pEle) const;
			void FromXMLElement(const Aris::Core::ELEMENT *pEle);

		};
		class MARKER :public OBJECT
		{
			friend class PART;
			friend class MODEL;
			friend struct std::pair < const std::string, MARKER >;
			friend struct std::pair < std::string, MARKER >;

		private:
			double _Pm[4][4];
			PART *_pPrt;
			double _PrtPm[4][4];

		private:
			MARKER(MODEL *pModel , const std::string &Name = "", PART* pPart = 0, const double *pLocPm = 0, MARKER *pRelativeTo = 0);
			
			MARKER(const MARKER &) = delete;
			MARKER(MARKER &&) = delete;
			MARKER& operator=(const MARKER &) = delete;
			MARKER& operator=(MARKER &&) = delete;

		public:
			~MARKER() = default;



		public:
			void Update();

			const double* GetPrtPmPtr() const;
			const double* GetPmPtr() const{ return *_Pm; };
			const double* GetVelPtr() const{ return _pPrt->GetVelPtr(); };
			const double* GetAccPtr() const{ return _pPrt->GetAccPtr(); };
			const PART* GetFatherPrt() const{ return _pPrt; };

			unsigned GetID() const;

			int ToXMLElement(Aris::Core::ELEMENT *pEle) const;
			int FromXMLElement(const Aris::Core::ELEMENT *pEle);
		};
		class JOINT :public OBJECT
		{
			friend class MODEL;
			friend class std::vector < JOINT >;
			friend class std::allocator < JOINT >;
			friend struct std::pair < const std::string, JOINT >;
			friend struct std::pair < std::string, JOINT >;

		public:
			enum JOINT_TYPE
			{
				ROTATIONAL,
				PRISMATIC,
				UNIVERSAL,
				SPHERICAL
			};
		private:
			JOINT_TYPE _Type;
			MARKER *_pMakI;
			MARKER *_pMakJ;

			int _ColId;

			/*for calculation in part coordinate*/
		private:
			double _tm_I2M[6][6];

			double _PrtCstMtxI[6][6];
			double _PrtCstMtxJ[6][6];
			double _CstFce[6];
			double _a_c[6];
		
		private:
			void _Initiate();

		private:
			JOINT(MODEL *pModel , const std::string &Name = "", JOINT_TYPE Type = ROTATIONAL, MARKER *pMakI = 0, MARKER *pMakJ = 0);
			
			JOINT(const JOINT &) = delete;
			JOINT(JOINT &&) = delete;
			JOINT& operator=(const JOINT &) = delete;
			JOINT& operator=(JOINT &&) = delete;

		public:
			~JOINT() = default;

		public:
			int GetCstDim() const;

			const MARKER* GetMakI() const{ return _pMakI; };
			const MARKER* GetMakJ() const{ return _pMakJ; };
			const double* GetCstFcePtr() const{ return _CstFce; };
			const double* GetPrtCstMtxIPtr() const{ return *_PrtCstMtxI; };
			const double* GetPrtCstMtxJPtr() const{ return *_PrtCstMtxJ; };
			const double* GetPrtA_cPtr() const{ return _a_c; };
			JOINT_TYPE GetType() const{ return _Type; };

			MARKER* GetMakI() { return _pMakI; };
			MARKER* GetMakJ() { return _pMakJ; };

			void UpdateInPrt();

			unsigned GetID() const;

			int ToXMLElement(Aris::Core::ELEMENT *pEle) const;
			int FromXMLElement(const Aris::Core::ELEMENT *pEle);
		};
		class MOTION :public OBJECT
		{
			friend class MODEL;
			friend struct std::pair < const std::string, MOTION >;
			friend struct std::pair < std::string, MOTION >;

		public:
			enum MOTION_TYPE
			{
				LINEAR
			};
			enum MOTION_MODE
			{
				POS_CONTROL,
				FCE_CONTROL
			};

		private:
			MOTION_TYPE _Type;
			MOTION_MODE _Mode;
			MARKER *_pMakI, *_pMakJ;

		private:
			double _p_m[6];
			double _v_m[6];
			double _a_m[6];
			double _f_m[6];
			double _frc_coe[3];

			int _ColId;

		private:
			double _PrtCstMtxI[6][6];
			double _PrtCstMtxJ[6][6];
			double _a_c[6];

		private:
			void _Initiate();

		private:
			MOTION(MODEL *pModel , const std::string &Name = "", MOTION_TYPE type = LINEAR, MOTION_MODE mode = POS_CONTROL, MARKER *pMakI = 0, MARKER *pMakJ = 0);

			MOTION(const MOTION &) = delete;
			MOTION(MOTION &&) = delete;
			MOTION& operator=(const MOTION &) = delete;
			MOTION& operator=(MOTION &&) = delete;

		public:
			~MOTION() = default;

		public:
			MOTION_TYPE GetType() const{ return _Type; };
			MOTION_MODE GetMode() const{ return _Mode; };
			int GetCstDim() const;
			
			const double* GetFrcCoePtr() const{ return _frc_coe; };
			const double* GetF_mPtr() const{ return _f_m; };
			const double* GetA_mPtr() const{ return _a_m; };
			const double* GetV_mPtr() const{ return _v_m; };
			const double* GetP_mPtr() const{ return _p_m; };
			const double* GetPrtCstMtxIPtr() const{ return *_PrtCstMtxI; };
			const double* GetPrtCstMtxJPtr() const{ return *_PrtCstMtxJ; };
			const double* GetPrtA_cPtr() const{ return _a_c; };
			
			void SetP_m(const double *p_m){ memcpy(_p_m, p_m, sizeof(double)*GetCstDim()); };
			void SetV_m(const double *v_m){ memcpy(_v_m, v_m, sizeof(double)*GetCstDim()); };
			void SetA_m(const double *a_m){ memcpy(_a_m, a_m, sizeof(double)*GetCstDim()); };
			void SetF_m(const double *f_m){ memcpy(_f_m, f_m, sizeof(double)*GetCstDim()); };
			void SetMode(MOTION_MODE mode){ _Mode = mode; };
			void SetFrcCoe(const double *frc_coe){ memcpy(_frc_coe, frc_coe, sizeof(double)* 3 * GetCstDim()); };

			void UpdateInPrt();

			/*for adams*/
		private:
			std::vector<std::pair<double, double> > posCurve;
			std::vector<std::pair<double, double> > fceCurve;

		public:
			void SetPosAkimaCurve(const unsigned num, const double* time, const double *pos)
			{
				posCurve.clear();
				if (posCurve.capacity() < num)
					posCurve.reserve(num);

				for (unsigned i = 0; i < num; ++i)
				{
					posCurve.push_back(std::make_pair(time[i], pos[i]));
				}
			};
			void SetFceAkimaCurve(const unsigned num, const double* time, const double *fce)
			{
				fceCurve.clear();
				if (fceCurve.capacity() < num)
					fceCurve.reserve(num);

				for (unsigned i = 0; i < num; ++i)
				{
					fceCurve.push_back(std::make_pair(time[i], fce[i]));
				}
			};

			unsigned GetID() const;

			int ToXMLElement(Aris::Core::ELEMENT *pEle) const;
			int FromXMLElement(const Aris::Core::ELEMENT *pEle);
		};
		class FORCE :public OBJECT
		{
			friend class MODEL;
			friend struct std::pair < const std::string, FORCE >;
			friend struct std::pair < std::string, FORCE >;

		public:
			enum FORCE_TYPE
			{
				BODY2BODY
			};
		private:
			FORCE_TYPE _Type;

			PART *_pPrtI;
			PART *_pPrtJ;
			MARKER *_pMakA;
			MARKER *_pMakP;

			double _LocFce[6];
			double _Fce[6];

		private:
			FORCE(MODEL *pModel = nullptr, const std::string &Name = "", FORCE_TYPE type = BODY2BODY, PART *pPrtI = 0, PART *pPrtJ = 0, MARKER *pMakA = 0, MARKER *pMakP = 0, const double *force = 0);
			
			FORCE(const FORCE &) = delete;
			FORCE(FORCE &&) = delete;
			FORCE& operator=(const FORCE &) = delete;
			FORCE& operator=(FORCE &&) = delete;

		public:
			~FORCE() = default;

		public:
			void Update();

			double* GetFceMtxPtr() const;
			void SetFce(const double* pFce);

			unsigned GetID() const;

			int ToXMLElement(Aris::Core::ELEMENT *pEle) const;
			int FromXMLElement(const Aris::Core::ELEMENT *pEle);
		};

		class ENVIRONMENT:public OBJECT
		{
			friend class PART;
			friend class MODEL;

		private:
			double Gravity[6];

		private:
			ENVIRONMENT(MODEL *pModel);
			~ENVIRONMENT();

			int ToXMLElement(Aris::Core::ELEMENT *pEle) const;
			int FromXMLElement(const Aris::Core::ELEMENT *pEle);
		};

		class MODEL:public OBJECT
		{
			friend class ENVIRONMENT;
			friend class PART;
			friend class MOTION;
			friend class JOINT;
			friend class MARKER;
			friend class FORCE;

		private:
			unsigned int C_dim;//real dimension of constraint matrix
			unsigned int I_dim;//real dimension of inertia matrix

		protected:
			std::vector<std::shared_ptr<PART> > _parts;
			std::vector<std::shared_ptr<JOINT> > _joints;
			std::vector<std::shared_ptr<MOTION> > _motions;
			std::vector<std::shared_ptr<FORCE> > _forces;
			std::vector<std::shared_ptr<MARKER> > _markers;
			
		protected:
			std::vector<double> _C;
			std::vector<double> _I;

			std::vector<double> _f;
			std::vector<double> _a_c;

			std::vector<double> _D;
			std::vector<double> _b;

			std::vector<double> _s;
			std::vector<double> _x;

		public:
			double *C, *pI, *f, *a_c, *D, *b,*s,*x;

			ENVIRONMENT _Environment;

		public:
			PART* pGround;
		public:
			MODEL(const std::string & Name = "Model");
			~MODEL();

			MODEL(const MODEL &) = delete;
			MODEL(MODEL &&) = delete;
			MODEL& operator=(const MODEL &) = delete;
			MODEL& operator=(MODEL &&) = delete;

		public:
			PART* AddPart(const std::string & Name
				, const double *Im = nullptr
				, const double *pm = nullptr
				, const double *Vel = nullptr
				, const double *Acc = nullptr);
			JOINT* AddJoint(const std::string & Name
				, Aris::DynKer::JOINT::JOINT_TYPE type = JOINT::ROTATIONAL
				, MARKER* pmaki = nullptr
				, MARKER* pmakj = nullptr);
			MOTION* AddMotion(const std::string & Name
				, Aris::DynKer::MOTION::MOTION_TYPE type = MOTION::LINEAR
				, MOTION::MOTION_MODE mode = MOTION::POS_CONTROL
				, MARKER *pMakI = nullptr
				, MARKER *pMakJ = nullptr);
			FORCE* AddForce(const std::string & Name
				, FORCE::FORCE_TYPE Type = FORCE::BODY2BODY
				, PART *pprti = nullptr
				, PART *pprtj = nullptr
				, MARKER *pmaka = nullptr
				, MARKER *pmakp = nullptr
				, const double *fce = nullptr);


			const PART *GetPart(unsigned id) const;
			const JOINT *GetJoint(unsigned id) const;
			const MOTION *GetMotion(unsigned id) const;
			const FORCE *GetForce(unsigned id) const;
			const MARKER *GetMarker(unsigned id) const;
			PART *GetPart(unsigned id);
			JOINT *GetJoint(unsigned id);
			MOTION *GetMotion(unsigned id);
			FORCE *GetForce(unsigned id);
			MARKER *GetMarker(unsigned id);
			const PART *GetPart(const std::string &Name)const;
			const JOINT *GetJoint(const std::string &Name)const;
			const MOTION *GetMotion(const std::string &Name)const;
			const FORCE *GetForce(const std::string &Name)const;
			PART *GetPart(const std::string &Name);
			JOINT *GetJoint(const std::string &Name);
			MOTION *GetMotion(const std::string &Name);
			FORCE *GetForce(const std::string &Name);
			unsigned GetPartNum() const{ return _parts.size(); };
			unsigned GetMotionNum() const{ return _motions.size(); };
			unsigned GetJointNum() const{ return _joints.size(); };
			unsigned GetForceNum() const{ return _forces.size(); };
			unsigned GetMarkerNum() const{ return _markers.size(); };

			void ForEachPart(std::function<void(const PART*)> fun) const
			{
				for (auto&i : _parts)
				{
					fun(i.get());
				}
			}
			/*void ForEachMotion(std::function<void(MOTION*)> fun)
			{
				for (auto&i : _motions)
				{
					fun(i.get());
				}
			}*/
			void ForEachMotion(std::function<void(MOTION*)> fun) const
			{
				for (auto&i : _motions)
				{
					fun(i.get());
				}
			}

		public:
			void DynPre();
			void DynPrtMtx();
			void Dyn();

		public:
			void ClbEqnTo(double *&clb_d_ptr, double *&clb_b_ptr, unsigned int &clb_dim_m, unsigned int &clb_dim_n);

		public:
			Aris::Core::DOCUMENT XML_Doc;
			CALCULATOR calculator;

		public:
			void LoadXML(const char *filename);
			void SaveSnapshotXML(const char *filename) const;
			virtual void SaveAdams(const char *filename) const;
		};
	}
}




























#endif
