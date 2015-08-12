#ifndef GUARD_YWIMISSIRGBDYBZS
#define GUARD_YWIMISSIRGBDYBZS

#include <algorithm>
#include <cassert>

template<typename Type, unsigned int Length>
class CircularBuffer {
    Type buf_[Length];
    volatile uint32_t read_pos_;
    volatile uint32_t write_pos_;
public:
    CircularBuffer() :
        read_pos_(0),
        write_pos_(0) {
        assert(write_available() == Length);
    }
    
    bool write_one(Type const & x) {
        if((write_pos_ + Length) % (2 * Length) != read_pos_) {
            buf_[write_pos_ % Length] = x;
            write_pos_ = (write_pos_ + 1) % (2 * Length);
            return true;
        } else {
            return false;
        }
    }
    uint32_t write_available() const {
        return (read_pos_ - write_pos_ + 3*Length) % (2*Length);
    }
    
    bool read_one(Type & x) {
        if(read_pos_ != write_pos_) {
            x = buf_[read_pos_ % Length];
            read_pos_ = (read_pos_ + 1) % (2 * Length);
            return true;
        } else {
            return false;
        }
    }
    void read_skip(uint32_t count) {
        while(count--) {
            assert(read_pos_ != write_pos_);
            read_pos_ = (read_pos_ + 1) % (2 * Length);
        }
    }
    Type const * read_pointer() const {
        return buf_ + read_pos_ % Length;
    }
    uint32_t read_contiguous_available() const {
        return std::min(Length - read_pos_ % Length, (write_pos_ - read_pos_ + 2*Length) % (2*Length));
    }
};

#endif
