#ifndef GUARD_YWIMISSIRGBDYBZS
#define GUARD_YWIMISSIRGBDYBZS

template<typename Type, unsigned int Length>
class CircularBuffer {
    Type buf_[Length];
    volatile uint32_t read_pos_;
    volatile uint32_t write_pos_;
public:
    CircularBuffer() :
        read_pos_(0),
        write_pos_(0) {
    }
    bool write(Type const & x) {
        if((write_pos_ + Length) % (2 * Length) != read_pos_) {
            buf_[write_pos_ % Length] = x;
            write_pos_ = (write_pos_ + 1) % (2 * Length);
            return true;
        } else {
            return false;
        }
    }
    bool read(Type & x) {
        if(read_pos_ != write_pos_) {
            x = buf_[read_pos_ % Length];
            read_pos_ = (read_pos_ + 1) % (2 * Length);
            return true;
        } else {
            return false;
        }
    }
};

#endif
