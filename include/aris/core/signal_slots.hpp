#ifndef SIGNAL_SLOTS_HPP_
#define SIGNAL_SLOTS_HPP_

#include <aris_lib_export.h>

#include <chrono>
#include <functional>

#include <aris/core/object.hpp>

namespace aris::core {

    template<typename Val, typename Attribute = int>  // Val, Attribute must have default constructor
    class ARIS_API Signal {
    public:
        auto virtual attribute()const->const Attribute& { return attr_; }
        auto virtual setAttribute(const Attribute& attr)->void { attr_ = attr; }

        auto virtual operator()(const Val& val)->void {
            val_ = val;
            time_ = std::chrono::system_clock::now();       // TODO: Can be called in RT-Thread?
            is_triggered_ = true;
        }

        auto virtual trigger()->void {
            time_ = std::chrono::system_clock::now();       // TODO: Can be called in RT-Thread?
            is_triggered_ = true;
        }

        auto virtual val()const->const Val& { return val_; }
        auto virtual setVal(const Val& val)->void { val_ = val; }

        auto virtual time()const->const std::chrono::system_clock::time_point& { return time_; }
        auto virtual setTime(const std::chrono::system_clock::time_point time)->void { time_ = time; }

        auto virtual isTriggered()const->bool { return is_triggered_; }
        auto virtual reset()->void {
            is_triggered_ = false;
            time_ = std::chrono::system_clock::time_point(std::chrono::duration<int>(0));
        }

    public:
        explicit Signal(const Attribute &attr = Attribute(), const Val &val = Val()) { attr_ = attr; val_ = val; }
        virtual ~Signal() = default;
        ARIS_DEFINE_BIG_FOUR(Signal);
        
    protected:
        Attribute attr_;
        Val val_;
        bool is_triggered_{false};
        std::chrono::system_clock::time_point time_;
    };

    template<typename Val, typename Attribute = int>
    class ARIS_API NumericalSignal :public Signal<Val, Attribute> {
    public:
        auto virtual operator==(const NumericalSignal &t)const->bool { return this->val_ == t.val_; }
        auto virtual operator>(const NumericalSignal &t)const->bool { return this->val_ > t.val_; }
        auto virtual operator>=(const NumericalSignal &t)const->bool { return this->val_ >= t.val_; }
        auto virtual operator<(const NumericalSignal &t)const->bool { return this->val_ < t.val_; }
        auto virtual operator<=(const NumericalSignal &t)const->bool { return this->val_ <= t.val_; }

    public:
        explicit NumericalSignal(const Attribute &attr = Attribute(), const Val& val = Val()) : Signal<Val, Attribute>(attr, val) {}
        virtual ~NumericalSignal() = default;
        ARIS_DEFINE_BIG_FOUR(NumericalSignal);
    };

    template<typename Signal, typename StatisticsInfo = int>
    class ARIS_API SignalMonitor {
    public:
        auto virtual monitoringMethod()const->const std::function<bool(const Signal&)>& { return moni_method_; }
        auto virtual setMonitoringMethod(const std::function<bool(const Signal&)> &method)->void { moni_method_ = method; }
        auto virtual triggeredCallback()const->const std::function<void(const Signal&)>& { return cbk_; }
        auto virtual setTriggeredCallback(const std::function<void(const Signal&)> cbk)->void { cbk_ = cbk; }

        auto virtual statisticsInfo()const->const StatisticsInfo& { return info_; }
        auto virtual statisticsMethod()const->const std::function<void(StatisticsInfo&, const Signal&, bool)>& { return sta_method_; }
        auto virtual setStatisticsMethod(const std::function<void(StatisticsInfo&, const Signal&, bool)> &method)->void { sta_method_ = method;}

        auto virtual monitoring(const Signal &sig)->void {
            bool triggered = false;
            if (moni_method_) triggered  = moni_method_(sig);
            if (triggered && cbk_)  cbk_(sig);
            if (sta_method_) sta_method_(info_, sig, triggered);
        }

    public:
        explicit SignalMonitor(const std::function<bool(const Signal&)> &method = nullptr) { moni_method_ = method; }
        virtual ~SignalMonitor() = default;
        ARIS_DELETE_BIG_FOUR(SignalMonitor);

    protected:
        StatisticsInfo info_;
        std::function<bool(const Signal&)> moni_method_{nullptr};
        std::function<void(const Signal&)> cbk_{nullptr};
        std::function<void(StatisticsInfo&, const Signal&, bool)> sta_method_{nullptr};
    };

    template<typename Val>
    class ARIS_API SingleEdgeDetector {
    public:
        auto virtual method(const Val &val)->bool {
            bool left_incr = right_threshold_>left_threshold_? val<=left_threshold_ : val>=left_threshold_;
            bool right_incr = right_threshold_>left_threshold_? val>=right_threshold_ : val<=right_threshold_;

            if (left_incr) {
                right_counter_ = 0;
                left_counter_++;
            }

            if (right_incr) {
                right_counter_++;
            }

            if (left_counter_ >=left_hold_ && right_counter_>=right_hold_) {
                left_counter_ = 0;
                right_counter_ = 0;
                return true;
            }

            return false;
        }

    public:
        explicit SingleEdgeDetector(const Val &left_threshold, const Val &right_threshold, std::uint64_t left_hold = 1, std::uint64_t right_hold = 1) :
            left_threshold_(left_threshold), right_threshold_(right_threshold), left_hold_(left_hold), right_hold_(right_hold) {
            if (right_threshold==left_threshold || left_hold==0 || right_hold==0) {
                throw std::runtime_error("SingleEdgeDetector parameter is not resonable.");
            }
        }
        virtual ~SingleEdgeDetector() = default;
        ARIS_DEFINE_BIG_FOUR(SingleEdgeDetector);

    protected:
        const Val left_threshold_;
        const Val right_threshold_;
        const std::uint64_t left_hold_;
        const std::uint64_t right_hold_;

        std::uint64_t left_counter_{0}, right_counter_{0};
    };

    template<typename Val>
    class ARIS_API BothEdgeDetector {
    public:
        auto virtual method(const Val &val)->bool {
            if (rising_detector_->method(val) || falling_detector_->method(val)) {
                return true;
            } else {
                return false;
            }
        }

    public:
        explicit BothEdgeDetector(const Val &lower_threshold, const Val &upper_threshold, std::uint64_t lower_hold = 1, std::uint64_t upper_hold = 1) {
            if (upper_threshold<=lower_threshold || lower_hold==0 || upper_hold==0) {
                throw std::runtime_error("BothEdgeDetector parameter is not resonable.");
            }

            rising_detector_ = std::make_shared<SingleEdgeDetector<Val>>(lower_threshold, upper_threshold, lower_hold, upper_hold);
            falling_detector_ = std::make_shared<SingleEdgeDetector<Val>>(upper_threshold, lower_threshold, upper_hold, lower_hold);
            assert(rising_detector_);
            assert(falling_detector_);
        }

        virtual ~BothEdgeDetector() = default;

        BothEdgeDetector(const BothEdgeDetector &t) {
            rising_detector_ = std::make_shared<SingleEdgeDetector<Val>>(*t.rising_detector_);
            falling_detector_ = std::make_shared<SingleEdgeDetector<Val>>(*t.falling_detector_);
        }

        BothEdgeDetector& operator=(const BothEdgeDetector &t) {
            rising_detector_ = std::make_shared<SingleEdgeDetector<Val>>(*t.rising_detector_);
            falling_detector_ = std::make_shared<SingleEdgeDetector<Val>>(*t.falling_detector_);
            return *this;
        }

        BothEdgeDetector(BothEdgeDetector &&t) {
            rising_detector_ = std::move(t.rising_detector_);
            falling_detector_ = std::move(t.falling_detector_);
        }

        BothEdgeDetector& operator=(BothEdgeDetector &&t) {
            rising_detector_ = std::move(t.rising_detector_);
            falling_detector_ = std::move(t.falling_detector_);
            return *this;
        }

    protected:
        std::shared_ptr<SingleEdgeDetector<Val>> rising_detector_;
        std::shared_ptr<SingleEdgeDetector<Val>> falling_detector_;
    };

}   // namespace aris::core

#endif // SIGNAL_SLOTS_HPP_