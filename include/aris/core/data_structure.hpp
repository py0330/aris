#ifndef ARIS_CORE_DATA_STRUCTURE_HPP_
#define ARIS_CORE_DATA_STRUCTURE_HPP_

#include <aris_lib_export.h>

#include <atomic>
#include <vector>
#include <assert.h>
#include <thread>

namespace aris::core {
    enum class AccessStrategy : int {
        kAbandon = 0,
        kForce,
        kYield
    };

    template<typename T>
    class ARIS_API Queue {
    public:
        Queue() {}
        virtual ~Queue() {}

    public:
        auto virtual isFull()const->bool = 0;
        auto virtual isEmpty()const->bool = 0;
        auto virtual size()const->int = 0;
        auto virtual capacity()const->int = 0;
        auto virtual push(const T &val)->bool = 0;
        auto virtual pop(T &val)->bool = 0;
    };

    template<typename T>
    class ARIS_API LockFreeArrayQueue :public Queue<T> {
    public:
        auto virtual isFull()const->bool override { return size_.load(std::memory_order_acquire) == capacity_; }
        auto virtual isEmpty()const->bool override { return size_.load(std::memory_order_acquire) == 0; }
        auto virtual size()const->int override { return size_.load(std::memory_order_acquire); }
        auto virtual capacity()const->int override { return capacity_; }
        auto virtual push(const T &val)->bool override {
            return push(val, AccessStrategy::kAbandon);
        }
        auto virtual pop(T &val)->bool override {
            return pop(val, AccessStrategy::kAbandon);
        }

        auto virtual push(const T &val, AccessStrategy strategy)->bool {
            int rear = -1;
            while (true) {
                rear = rear_.exchange(kExclude_);
                if ( rear != kExclude_) {
                    if (isFull()) {
                        rear_.exchange(rear);
                    } else {
                        break;
                    }
                }

                switch (strategy) {
                case AccessStrategy::kAbandon :
                    return false;
                case AccessStrategy::kYield :
                    std::this_thread::yield();
                    break;
                case AccessStrategy::kForce :
                    break;
                default :
                    return false;
                }
            }

            data_[rear] = val;
            size_++;
            rear_.exchange((rear+1) & (capacity_-1));

            return true;
        }

        auto virtual pop(T &val, AccessStrategy strategy)->bool {
            int front = -1;
            while (true) {
                front = front_.exchange(kExclude_);
                if (front != kExclude_) {
                    if (isEmpty()) {
                        front_.exchange(front);
                    } else {
                        break;
                    }
                }

                switch (strategy) {
                case AccessStrategy::kAbandon :
                    return false;
                case AccessStrategy::kYield :
                    std::this_thread::yield();
                    break;
                case AccessStrategy::kForce :
                    break;
                }
            }

            val = data_[front];
            size_--;
            front_.exchange((front+1) & (capacity_-1));

            return true;
        }

    public:
        LockFreeArrayQueue() {
            new (this)LockFreeArrayQueue(16);
        }

        explicit LockFreeArrayQueue(int capacity) {
            static auto func_round_up_pow_of_two = [](int val)->int {
                if (val <= 0) return 1;
                if (val & (val-1) == 0) return val;

                int position = 0;
                for (int i=val; i!=0; i>>=1)
                    position++;

                return 1 << position;
            };

            capacity_ = func_round_up_pow_of_two(capacity);
            data_.resize(capacity_, T()); 
            size_.store(0, std::memory_order_release);
            front_.store(0, std::memory_order_release);
            rear_.store(0, std::memory_order_release);
        }

        ~LockFreeArrayQueue() = default;

        LockFreeArrayQueue(const LockFreeArrayQueue &q) {
            kExclude_ = q.kExclude_;
            data_ = q.data_;
            capacity_ = q.capacity_;
            size_.store(q.size_);
            front_.store(q.front_);
            rear_.store(q.rear_);
        }

        LockFreeArrayQueue& operator=(const LockFreeArrayQueue &q) {
            kExclude_ = q.kExclude_;
            data_ = q.data_;
            capacity_ = q.capacity_;
            size_.store(q.size_);
            front_.store(q.front_);
            rear_.store(q.rear_);

            return *this;
        }

        LockFreeArrayQueue(LockFreeArrayQueue &&q) {
            kExclude_ = q.kExclude_;
            data_ = std::move(q.data_);
            capacity_ = q.capacity_;
            size_.store(q.size_);
            front_.store(q.front_);
            rear_.store(q.rear_);
        }

        LockFreeArrayQueue& operator=(LockFreeArrayQueue &&q) {
            kExclude_ = q.kExclude_;
            data_ = std::move(q.data_);
            capacity_ = q.capacity_;
            size_.store(q.size_);
            front_.store(q.front_);
            rear_.store(q.rear_);

            return *this;
        }

    private:
        int kExclude_{-1};
        std::vector<T> data_;
        int capacity_;
        std::atomic<int> size_;
        std::atomic<int> front_;
        std::atomic<int> rear_;
    };
}   // namespace aris::core

#endif // ARIS_CORE_DATA_STRUCTURE_HPP_