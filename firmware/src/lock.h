#ifndef GUARD_KWSHZGLMLZZTMTLH
#define GUARD_KWSHZGLMLZZTMTLH

#include "coroutine.h"

class Lock {
public:
    class User;
private:
    User *last_;
public:
    Lock() :
        last_(nullptr) {
    }
    ~Lock() {
        assert(!last_);
    }
    Lock(Lock const &) = delete; Lock & operator=(Lock const &) = delete; // noncopyable

    class User {
        Lock & lock_;
        User * next_;
        CoroutineBase * coroutine_;
        void wake() {
            assert(!coroutine_->run_some());
        }
    public:
        User(Lock & lock) :
            lock_(lock),
            next_(nullptr),
            coroutine_(current_coroutine) {
            if(!lock.last_) {
                lock.last_ = this;
            } else {
                assert(!lock.last_->next_);
                lock.last_->next_ = this;
                lock.last_ = this;
                yield();
            }
        }
        ~User() {
            if(next_) {
                next_->wake();
            } else {
                assert(lock_.last_ == this);
                lock_.last_ = nullptr;
            }
        }
        User(User const &) = delete; User & operator=(User const &) = delete; // noncopyable
    };
};

#endif
