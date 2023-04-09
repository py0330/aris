#ifndef ARIS_DYNAMIC_MODEL_MOTION_H_
#define ARIS_DYNAMIC_MODEL_MOTION_H_

#include <cmath>

#include <aris/dynamic/model_interaction.hpp>

namespace aris::dynamic {
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	class ARIS_API MotionBase :public Constraint {
	public:
		auto virtual eeType()const->EEType { return EEType::UNKNOWN; }
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void;
		auto virtual cptCp(double* cp)const noexcept->void override { cptCpFromPm(cp, *makI()->pm(), *makJ()->pm(), p()); }

		auto virtual cptPFromPm(const double* pm_i2j, double* p)const noexcept->void {}
		auto virtual cptPmFromP(const double* p, double* pm_i2j)const noexcept->void {}
		auto virtual compareP(const double* p1, const double* p2)->double; // 比较两个输入差，返回最大的差值
		auto virtual pSize()const noexcept->Size { return dim(); }
		auto virtual p()const noexcept->const double* { return nullptr; }
		auto virtual updP() noexcept->void;
		auto virtual setPByMak(const Marker* mak_i, const Marker* mak_j, const double* p) noexcept->void;
		auto virtual setP(const double *p) noexcept->void {}
		auto virtual getP(double *p)const noexcept->void { s_vc(pSize(), this->p(), p); }
		auto virtual vSize()const noexcept->Size { return dim(); }
		auto virtual v()const noexcept->const double* { return nullptr; }
		auto virtual updV() noexcept->void {}
		auto virtual setV(const double *v) noexcept->void {}
		auto virtual getV(double *v)const noexcept->void { s_vc(vSize(), this->v(), v); }
		auto virtual aSize()const noexcept->Size { return dim(); }
		auto virtual a()const noexcept->const double* { return nullptr; }
		auto virtual updA() noexcept->void {}
		auto virtual setA(const double *a) noexcept->void {}
		auto virtual getA(double *a)const noexcept->void { s_vc(aSize(), this->a(), a); }
		auto virtual fSize()const noexcept->Size { return dim(); }
		auto virtual f()const noexcept->const double* { return cf(); }
		auto virtual setF(const double* mf) noexcept->void { setCf(mf); }
		auto virtual getF(double *f)const noexcept->void { s_vc(fSize(), this->f(), f); }

		virtual ~MotionBase() = default;
		explicit MotionBase(const std::string &name = "motion_base", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) 
		: Constraint(name, makI, makJ, active){}
		ARIS_DEFINE_BIG_FOUR(MotionBase);
	};


	template<int P_SIZE, int V_SIZE, int A_SIZE>
	class ARIS_API MotionTemplate : public MotionBase {
	public:
		auto virtual pSize()const noexcept->Size override { return P_SIZE; }
		auto virtual p()const noexcept->const double* { return p_; }
		auto virtual setP(const double* p) noexcept->void override { s_vc(pSize(), p, this->p_); }
		auto virtual getP(double* p)const noexcept->void override { s_vc(pSize(), this->p_, p); }
		auto virtual vSize()const noexcept->Size override { return V_SIZE; }
		auto virtual v()const noexcept->const double* override { return v_; }
		auto virtual setV(const double* v) noexcept->void override { s_vc(vSize(), v, this->v_); }
		auto virtual getV(double* v)const noexcept->void override { s_vc(vSize(), this->v_, v); }
		auto virtual aSize()const noexcept->Size override { return A_SIZE; }
		auto virtual a()const noexcept->const double* override { return a_; }
		auto virtual setA(const double* a) noexcept->void override { s_vc(aSize(), a, this->a_); }
		auto virtual getA(double* a)const noexcept->void override { s_vc(aSize(), this->a_, a); }


		virtual ~MotionTemplate() = default;
		explicit MotionTemplate(const std::string& name = "motion_template", Marker* makI = nullptr, Marker* makJ = nullptr, bool active = true) 
			:MotionBase(name, makI, makJ, active){}

	protected:
		double p_[P_SIZE]{ 0.0 }, v_[V_SIZE]{ 0.0 }, a_[A_SIZE]{ 0.0 };
	};

	// 单位末端，电机或直线电机 //
	class ARIS_API Motion final :public MotionBase{
	public:
		static auto Dim()->Size { return 1; }
		auto virtual eeType()const->EEType override { return axis() < 3 ? EEType::X : EEType::A; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void override;
		auto virtual cptCv(double *cv)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;
		auto virtual cptPFromPm(const double* pm_i2j, double* p)const noexcept->void override;
		auto virtual cptPmFromP(const double* p, double* pm_i2j)const noexcept->void override;
		auto virtual compareP(const double* p1, const double* p2)->double;
		auto virtual p() const noexcept->const double* override;
		auto virtual updP() noexcept->void override;
		auto virtual setP(const double *mp) noexcept->void override;
		auto virtual v()const noexcept->const double* override;
		auto virtual updV() noexcept->void override;
		auto virtual setV(const double *mp) noexcept->void override;
		auto virtual a()const noexcept->const double* override;
		auto virtual updA() noexcept->void override;
		auto virtual setA(const double *mp) noexcept->void override;
		auto virtual f()const noexcept->const double* override;
		auto virtual setF(const double *mf) noexcept->void override;

		// mp          = mp_internal / mp_offset - mp_offset
		// mp_internal = (mp + mp_offset) * mp_factor
		auto mp()const noexcept->double { return *p(); }
		auto setMp(double mp) noexcept->void { setP(&mp); }
		auto mv() const noexcept->double { return *v(); }
		auto setMv(double mv) noexcept->void { setV(&mv); }
		auto ma() const noexcept->double { return *a(); }
		auto setMa(double ma) noexcept->void { setA(&ma); }
		auto mf() const noexcept->double { return *f(); }
		auto setMf(double mf) noexcept->void { setF(&mf); }

		// 驱动轴，可以为0~5
		// 0-2： 为 x-z 方向的平移
		// 3-5： 为 x-z 方向的转动
		auto setAxis(Size axis)noexcept->void;
		auto axis()const noexcept->Size;

		// 导程
		// 表示每转一圈，行进距离，单位是 m/round
		// 只有在axis() 为 3-5时才有用
		// 当pitch不为0时，不会考虑
		auto pitch()const noexcept->double;
		auto setPitch(double pitch)noexcept->void;

		// 仅仅影响  updP() 函数计算转动轴的转角 mp 所处的象限   【注意是mp，不是mpInternal】
		//
		// 经过 upd() 函数调用后，mp 转角所处象限为：
		// rotate range 为 0（默认值）  ：【-p/2        , p/2         】
		// rotate range 为 r            ：【-p/2 + p*r  , p + p*r     】
		// rotate range 为 nan          ： 距离当前转角最近的角度
		//
		// 以上 p 是指完整的角度周期，和 mpFator 有关。
		// 例如当 mpFactor() = 0      时，周期 p 是 2 * pi
		//     当 mpFactor() = pi/180 时，周期 p 是 360 
		auto setRotateRange(double range)noexcept->void;
		auto rotateRange()const noexcept->double;
		
		auto mfDyn() const noexcept->double;
		auto setMfDyn(double mf_dyn) noexcept->void;
		auto mfFrc() const noexcept->double;
		auto frcCoe() const noexcept ->const double3&;
		auto setFrcCoe(const double *frc_coe) noexcept->void;
		auto frcZeroCheck()const noexcept ->double { return 1e-2; }
		
		// 位置偏移
		auto mpOffset()const noexcept->double;
		auto setMpOffset(double mp_offset)noexcept->void;
		
		// 位置系数，默认为1，此时单位是m 或 rad
		auto mpFactor()const noexcept->double;
		auto setMpFactor(double mp_factor)noexcept->void;
		
		// mp          = mp_internal / mp_offset - mp_offset
		// mp_internal = (mp + mp_offset) * mp_factor
		auto mpInternal()const noexcept->double;
		auto setMpInternal(double mp_internal)noexcept->void;



		virtual ~Motion();
		explicit Motion(const std::string &name = "motion", Marker *makI = nullptr, Marker *makJ = nullptr
			, Size component_axis = 2, const double *frc_coe = nullptr, double mp_offset = 0.0, double mp_factor = 1.0
			, bool active = true);
		ARIS_DECLARE_BIG_FOUR(Motion);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	// 六维末端
	class ARIS_API GeneralMotion final :public MotionBase{
	public:
		enum class PoseType{
			EULER313,
			EULER321,
			EULER123,
			QUATERNION,
			POSE_MATRIX,
		};
		enum class VelType {
			VEL,
			VEL_SCREW,
		};
		enum class AccType {
			ACC,
			ACC_SCREW,
		};
		enum class FceType {
			FCE,
			FCE_SCREW,
		};

		auto setPoseType(PoseType type)->void;
		auto poseType()const->PoseType;
		auto setVelType(VelType type)->void;
		auto velType()const->VelType;
		auto setAccType(AccType type)->void;
		auto accType()const->AccType;
		auto setFceType(FceType type)->void;
		auto fceType()const->FceType;

		static auto Dim()->Size { return 6; }
		auto virtual eeType()const->EEType override;
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptCv(double *cv)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;
		auto virtual cptPFromPm(const double* pm_i2j, double* p)const noexcept->void override;
		auto virtual cptPmFromP(const double* p, double* pm_i2j)const noexcept->void override;
		auto virtual pSize()const noexcept->Size;
		auto virtual p()const noexcept->const double* override;
		auto virtual updP() noexcept->void override;
		auto virtual setP(const double* mp) noexcept->void override;
		auto virtual getP(double* mp)const noexcept->void override;
		auto virtual vSize()const noexcept->Size override;
		auto virtual v()const noexcept->const double* override;
		auto virtual updV() noexcept->void override;
		auto virtual setV(const double* mv) noexcept->void override;
		auto virtual getV(double* mv)const noexcept->void override;
		auto virtual aSize()const noexcept->Size override;
		auto virtual a()const noexcept->const double* override;
		auto virtual updA() noexcept->void override;
		auto virtual setA(const double* ma) noexcept->void override;
		auto virtual getA(double* ma)const noexcept->void override;

		auto mpm()const noexcept->const double4x4&;
		auto setMpe(const double* pe, const char *type = "313") noexcept->void;
		auto setMpq(const double* pq) noexcept->void;
		auto setMpm(const double* pm) noexcept->void;
		auto getMpe(double* pe, const char *type = "313")const noexcept->void;
		auto getMpq(double* pq)const noexcept->void;
		auto getMpm(double* pm)const noexcept->void;
		auto mvs()const noexcept->const double6&;
		auto setMve(const double* ve, const char *type = "313") noexcept->void;
		auto setMvq(const double* vq) noexcept->void;
		auto setMvm(const double* vm) noexcept->void;
		auto setMva(const double* va) noexcept->void;
		auto setMvs(const double* vs) noexcept->void;
		auto getMve(double* ve, const char *type = "313")const noexcept->void;
		auto getMvq(double* vq)const noexcept->void;
		auto getMvm(double* vm)const noexcept->void;
		auto getMva(double* va)const noexcept->void;
		auto getMvs(double* vs)const noexcept->void;
		auto mas()const noexcept->const double6&;
		auto setMae(const double* ae, const char *type = "313") noexcept->void;
		auto setMaq(const double* aq) noexcept->void;
		auto setMam(const double* am) noexcept->void;
		auto setMaa(const double* aa) noexcept->void;
		auto setMas(const double* as) noexcept->void;
		auto getMae(double* ae, const char *type = "313")const noexcept->void;
		auto getMaq(double* aq)const noexcept->void;
		auto getMam(double* am)const noexcept->void;
		auto getMaa(double* aa)const noexcept->void;
		auto getMas(double* as)const noexcept->void;

		virtual ~GeneralMotion();
		explicit GeneralMotion(const std::string &name = "general_motion", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true);
		ARIS_DECLARE_BIG_FOUR(GeneralMotion);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	// 只包含 xyz 3个维度的末端，例如 4足 机器人的足端
	class ARIS_API PointMotion final :public MotionTemplate<3, 3, 3>{
	public:
		static auto Dim()->Size { return 3; }
		auto virtual eeType()const->EEType override { return EEType::XYZ; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void override;
		auto virtual cptCv(double* cv)const noexcept->void override;
		auto virtual cptCa(double* ca)const noexcept->void override;
		auto virtual cptPFromPm(const double* pm_i2j, double* p)const noexcept->void override;
		auto virtual cptPmFromP(const double* p, double* pm_i2j)const noexcept->void override;
		auto virtual updV() noexcept->void override;
		auto virtual updA() noexcept->void override;
		auto virtual f()const noexcept->const double* override { return cf(); }
		auto virtual setF(const double* mf) noexcept->void override { setCf(mf); }

		virtual ~PointMotion();
		explicit PointMotion(const std::string& name = "point_motion", Marker* makI = nullptr, Marker* makJ = nullptr, bool active = true);
		ARIS_DECLARE_BIG_FOUR(PointMotion);
	};
	// 只包含 xyz 3个维度的末端，例如 4足 机器人的足端
	class ARIS_API SphericalMotion final :public MotionTemplate<3, 3, 3>{
	public:
		static auto Dim()->Size { return 3; }
		auto virtual eeType()const->EEType override { return EEType::ABC; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void override;
		auto virtual cptCv(double* cv)const noexcept->void override;
		auto virtual cptCa(double* ca)const noexcept->void override;
		auto virtual cptPFromPm(const double* pm_i2j, double* p)const noexcept->void override;
		auto virtual cptPmFromP(const double* p, double* pm_i2j)const noexcept->void override;
		auto virtual updV() noexcept->void override;
		auto virtual updA() noexcept->void override;
		auto virtual f()const noexcept->const double* override { return cf(); }
		auto virtual setF(const double* mf) noexcept->void override { setCf(mf); }

		virtual ~SphericalMotion();
		explicit SphericalMotion(const std::string& name = "spherical_motion", Marker* makI = nullptr, Marker* makJ = nullptr, bool active = true);
		ARIS_DECLARE_BIG_FOUR(SphericalMotion);
	};
	// 只包含 xyz 和 theta 4个维度的末端，例如 scara和delta的末端
	class ARIS_API XyztMotion final :public MotionTemplate<4, 4, 4>{
	public:
		static auto Dim()->Size { return 4; }
		auto virtual eeType()const->EEType override { return EEType::XYZT; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm, const double* mp)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptCv(double *cv)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;
		auto virtual cptPFromPm(const double* pm_i2j, double* p)const noexcept->void override;
		auto virtual cptPmFromP(const double* p, double* pm_i2j)const noexcept->void override;
		auto virtual updV() noexcept->void override;
		auto virtual updA() noexcept->void override;

		// 仅仅影响  updP() 函数计算转动轴的转角 mp 所处的象限   【注意是mp，不是mpInternal】
		//
		// 经过 upd() 函数调用后，mp 转角所处象限为：
		// rotate range 为 0（默认值）  ：【-p/2        , p/2         】
		// rotate range 为 r            ：【-p/2 + p*r  , p + p*r     】
		// rotate range 为 nan          ： 距离当前转角最近的角度
		//
		// 以上 p 是指完整的角度周期，和 mpFator 有关。
		// 例如当 mpFactor() = 0      时，周期 p 是 2 * pi
		//     当 mpFactor() = pi/180 时，周期 p 是 360 
		auto setRotateRange(double range)noexcept->void;
		auto rotateRange()const noexcept->double;

		virtual ~XyztMotion();
		explicit XyztMotion(const std::string &name = "xyzt_motion", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true);
		ARIS_DECLARE_BIG_FOUR(XyztMotion);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	// 只包含 xy 和 theta 的平面运动末端
	class ARIS_API PlanarMotion final :public MotionTemplate<3, 3, 3>{
	public:
		static auto Dim()->Size { return 3; }
		auto virtual eeType()const->EEType override { return EEType::XYT; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void override;
		auto virtual cptCv(double* cv)const noexcept->void override;
		auto virtual cptCa(double* ca)const noexcept->void override;
		auto virtual cptPFromPm(const double* pm_i2j, double* p)const noexcept->void override;
		auto virtual cptPmFromP(const double* p, double* pm_i2j)const noexcept->void override;
		auto virtual updV() noexcept->void override;
		auto virtual updA() noexcept->void override;

		virtual ~PlanarMotion();
		explicit PlanarMotion(const std::string& name = "planar_motion", Marker* makI = nullptr, Marker* makJ = nullptr, bool active = true);
		ARIS_DECLARE_BIG_FOUR(PlanarMotion);
	};
	// 只包含 xy 和 theta 的平面运动末端
	class ARIS_API XyMotion final :public MotionTemplate<2, 2, 2>{
	public:
		static auto Dim()->Size { return 2; }
		auto virtual eeType()const->EEType override { return EEType::XY; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void override;
		auto virtual cptCv(double* cv)const noexcept->void override;
		auto virtual cptCa(double* ca)const noexcept->void override;
		auto virtual cptPFromPm(const double* pm_i2j, double* p)const noexcept->void override;
		auto virtual cptPmFromP(const double* p, double* pm_i2j)const noexcept->void override;
		auto virtual updV() noexcept->void override;
		auto virtual updA() noexcept->void override;

		virtual ~XyMotion();
		explicit XyMotion(const std::string& name = "xy_motion", Marker* makI = nullptr, Marker* makJ = nullptr, bool active = true);
		ARIS_DECLARE_BIG_FOUR(XyMotion);
	};

	/// @}
}

#endif
