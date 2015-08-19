#ifndef GUARD_VWKUDDVSPUUAMFRC
#define GUARD_VWKUDDVSPUUAMFRC

class RunnerBase {
public:
    virtual void run() const = 0;
    virtual ~RunnerBase() {
    };
};

template<typename Function>
class Runner : public RunnerBase {
    Function const & func_;
public:
    Runner(Function const & func) :
        func_(func) {
    }
    void run() const {
        func_();
    }
    ~Runner() {
    }
};

#endif
