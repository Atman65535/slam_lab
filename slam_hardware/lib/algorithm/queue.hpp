#pragma once
#include <stdint.h>
// #include <stm32f4xx_hal.h>


template <typename T, uint32_t Size> 
class Queue {
public:
    Queue():count(0), size(Size), head(0), tail(0), busy(false) {}
    ~Queue() = default;

    bool in_queue(const T sample) {
        if (count == size)
            return false;
        queue[tail] = sample;
        tail = (tail + 1) % size;
        count ++;
        return true;
    }
    
    T out_queue(void) {
        // assert count != 0
        T ret = queue[head];
        head = (head + 1) % size;
        count -= 1;
        return ret;
    }

    bool is_full(void) {
        return count == size;
    }

    bool is_empty(void) {
        return count == 0;
    }
    uint32_t get_count() {
        return count;
    }
    void set_busy() {
        busy = true;
    }
    void set_free() {
        busy = false;
    }
    bool is_busy() {
        return busy;
    }
private:
    T queue[Size];
    const uint32_t size;
    uint32_t count;
    uint32_t head; // to pop
    uint32_t tail; // to push
    bool busy;
};
