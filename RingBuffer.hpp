#pragma once

#include <cstdint>
#include <cstddef>
#include <sys/mman.h>

#include <unistd.h>

#include <iostream>
#include <cstring>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <stdexcept>
#include <cassert>

#include <atomic>

template <typename T>
class RingBuffer {

    //const void *buffer_;
    T* buffer_;
    uint32_t capacity_;

    std::atomic<uint32_t> read_pos_;
    std::atomic<uint32_t> write_pos_;

    std::mutex lock_;
    std::condition_variable produced_cond_;
    std::condition_variable consumed_cond_;


    // Unmap mirror memory
    static void unmap_mirror(const void *addr, const size_t size) noexcept {
        int munmap_res;
        munmap_res = munmap(const_cast<void *>(addr), 2 * size);
        if(munmap_res == -1) {
            std::cerr << "munmap failed" << std::endl;
        }
    }

    static bool is_power_of_two(const uint32_t val) {
        return (val != 0) && ((val & (val - 1)) == 0);
    }

    // Double mapped memory for "magic" ringbuffer.
    static T* map_mirror(uint32_t size) {
        // Get pagesize
        const uint32_t pagesize = getpagesize();
        if(size < pagesize) {
            throw std::runtime_error("Capacity must be at least pagesize: " + std::to_string(pagesize)
                + " requested size: " + std::to_string(size));
        }

        if(!is_power_of_two(size)) {
            throw std::runtime_error("Capacity must be a power of two, requested size: " + std::to_string(size));
        }

        // Create a memfd. Name is only for debugging purposes and can
        // be reused.
        int mem_fd = memfd_create("soapy_ring_buffer", 0);
        if(mem_fd == -1) {
            // TODO: add excact error
            throw std::runtime_error("Could not create memfd: " + std::string(strerror(errno)));
        }

        // Truncate to size
        const int ftruncate_res = ftruncate(mem_fd, size);
        if(ftruncate_res == -1) {
            throw std::runtime_error("Could not ftruncate memfd: " + std::string(strerror(errno)));
        }

        // Find a piece of memory of size 2 * size.
        void* buffer = mmap(NULL, 2 * size, PROT_NONE, MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
        if (buffer == MAP_FAILED) {
            throw std::runtime_error("Could not mmap buffer: " + std::string(strerror(errno)));
        }

        // Map the memfd to the first half of the buffer
        void* addr_hint_1 = buffer;
        void* buffer_1 = mmap(addr_hint_1, size,
                              PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED,
                              mem_fd, 0);

        if (buffer_1 == MAP_FAILED or buffer_1 != addr_hint_1) {
            throw std::runtime_error("Could not mmap buffer1: " + std::string(strerror(errno)));
        }

        // Map the memfd to the second half of the buffer
        // The double cast is because of pointer arithmetic
        // TODO: fix this cast into a macro or something
        void* addr_hint_2 = (void*)((uint8_t*)buffer_1 + size);
        const void* buffer_2 = mmap(addr_hint_2, size,
                                    PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED,
                                    mem_fd, 0);

        // Check if the second mmap was successful and that address is correct.
        if (buffer_2 == MAP_FAILED or buffer_2 != addr_hint_2) {
            throw std::runtime_error("Could not mmap buffer2: " + std::string(strerror(errno)));
        }

        // Probably not strictly neccassry.
        std::memset(buffer, 0, size);

        // All correct, return a pointer to the buffer
        return static_cast<T*>(buffer);
    }

    // Mask read and write pointer
    inline uint32_t mask(const uint32_t val) const noexcept {
        return (capacity_ - 1) & val;
    }

    public:

    size_t size() const noexcept {
        return capacity_ * sizeof(T);
    }

    inline uint32_t capacity() const noexcept {
        return capacity_;
    }

    inline void produce(const uint32_t elements) noexcept {
        // intentional wrap around arithmetic
        write_pos_ += elements;
        produced_cond_.notify_one();
    }

    inline void consume(const uint32_t elements) noexcept {
        // intentional wrap around arithmetic
        read_pos_ += elements;
        consumed_cond_.notify_one();
    }

    inline uint32_t available() noexcept {
        return (write_pos_ - read_pos_);
    }

    inline uint32_t free() noexcept {
        return capacity() - available();
    }

    inline const T* read_ptr() const noexcept {
        return buffer_ + mask(read_pos_);
    }

    inline T* write_ptr() const noexcept {
        return buffer_ + mask(write_pos_);
    }

    void clear() noexcept {
        // std::unique_lock<std::mutex> lock(lock_);
        read_pos_ = 0;
        write_pos_ = 0;
        // wake up producer
        // no need to wake up consumer since there's nothing to consume
        consumed_cond_.notify_all();
    }

    int32_t read_at_least(uint32_t elements,
                       std::chrono::microseconds timeout,
                       std::function<uint32_t(const T* begin, const uint32_t avail)> callback) {

        std::unique_lock<std::mutex> lock(lock_);
        // Wait for enough data to be available
        if(produced_cond_.wait_for(lock, timeout, [this, elements] {
            return available() >= elements; }))
        {
            // Ok, we have enough data
            uint32_t consumed = callback(read_ptr(), available());
            consume(consumed);
            return consumed;
        } else {
            // Timeout
            return -1;
        }
    }


    int32_t write_at_least(uint32_t elements,
                       std::chrono::microseconds timeout,
                       std::function<uint32_t(T* begin, const uint32_t free)> callback) {

        std::unique_lock<std::mutex> lock(lock_);
        // Wait for enough data to be available
        if(consumed_cond_.wait_for(lock, timeout, [this, elements] {
            return free() >= elements; }))
        {
            // Ok, we have enough data
            uint32_t produced = callback(write_ptr(), free());
            produce(produced);
            return produced;
        } else {
            // Timeout
            return -1;
        }
    }

    RingBuffer(uint32_t capacity)
        : buffer_(map_mirror(capacity * sizeof(T))),
          capacity_(capacity),
          read_pos_(0),
          write_pos_(0) {
    }

    virtual ~RingBuffer() {
        unmap_mirror(buffer_, capacity_ * sizeof(T));
    };

};
