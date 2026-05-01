// Created by wyb on 17-6-27.
// Modified (add template) by Weifan Gao on 23-04-30 and Liran on 26-04-28

#ifndef RADAR_DRIVER_CPP_KFIFO_UNLOCK_QUEUE_H_
#define RADAR_DRIVER_CPP_KFIFO_UNLOCK_QUEUE_H_

#include <stdint.h>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

inline bool is_power_of_2(uint32_t num) {
    return (num != 0 && (num & (num - 1)) == 0);
}

inline uint32_t highest_one_bit(uint32_t num) {
    num |= (num >> 1);
    num |= (num >> 2);
    num |= (num >> 4);
    num |= (num >> 8);
    num |= (num >> 16);
    return num - (num >> 1);
}

inline uint32_t roundup_pow_of_two(uint32_t num) {
    return num > 1 ? highest_one_bit((num - 1) << 1) : 1;
}

template <class T>
class UnlockQueue {
public:
    explicit UnlockQueue(uint32_t size)
        : _buffer(new T[roundup_pow_of_two(size)]),
          _size(roundup_pow_of_two(size)),
          _in(0),
          _out(0) {
    }

    UnlockQueue(const UnlockQueue&) = delete;
    UnlockQueue& operator=(const UnlockQueue&) = delete;

    UnlockQueue(UnlockQueue&&) = delete;
    UnlockQueue& operator=(UnlockQueue&&) = delete;

    ~UnlockQueue() {
        if (_buffer) {
            delete[] _buffer;
        }
    }

    uint32_t Put(const T* buffer, uint32_t len) {
        if (len > _size - (_in - _out.load(std::memory_order_acq_rel))) {
            return 0;
        }

        const uint32_t write_offset = _in & (_size - 1);
        const uint32_t first_copy_count = std::min(len, _size - write_offset);
        memcpy(_buffer + write_offset, buffer, first_copy_count * sizeof(T));
        memcpy(_buffer, buffer + first_copy_count, (len - first_copy_count) * sizeof(T));
        _in.fetch_add(len, std::memory_order_release);
        return len;
    }

    uint32_t Get(T* buffer, uint32_t len) {
        len = std::min(len, _in.load(std::memory_order_acquire) - _out);

        const uint32_t read_offset = _out & (_size - 1);
        const uint32_t first_copy_count = std::min(len, _size - read_offset);
        memcpy(buffer, _buffer + read_offset, first_copy_count * sizeof(T));
        memcpy(buffer + first_copy_count, _buffer, (len - first_copy_count) * sizeof(T));
        _out.fetch_add(len, std::memory_order_acq_rel);
        return len;
    }

    uint32_t Get_wait(T* buffer, uint32_t len, uint32_t timeout_ms) {
        uint32_t wait_count = 0;
        while (length() < len) {
            if (wait_count++ >= timeout_ms) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return Get(buffer, len);
    }

    uint32_t size() const { return _size; }
    uint32_t length() const { return (_in - _out); }
    bool empty() const { return _in <= _out; }

private:
    T* _buffer;
    uint32_t _size;
    std::atomic<uint32_t> _in;
    std::atomic<uint32_t> _out;
};

#endif  // RADAR_DRIVER_CPP_KFIFO_UNLOCK_QUEUE_H_
