//
// Copyright 2020 Marco Cominelli
//

#ifndef DOUBLEBUFFER_HPP
#define DOUBLEBUFFER_HPP

#include <atomic>
#include <cstdint>


//---------------------------
// Template class definition
//---------------------------

template <typename T>
class DoubleBuffer
{
public:
    DoubleBuffer();
    ~DoubleBuffer();

    // Writing methods.
    T* start_writing(); // never returns nullptr
    void end_writing();

    // Reading methods.
    T* start_reading(); // returns nullptr is there is no new data
    void end_reading();

    T m_buf[2];
private:

    // The lowest bit will is the active cell (used for writing). The active
    // cell can only be switched if there is at most one concurrent user.
    // The next two bits of state will be the number of concurrent users.
    // The fourth bit indicates if there is value available for reading in
    // m_buf[0] and the fifth bit has the same meaning for m_buf[1].
    std::atomic<std::uint32_t> m_state;

    std::uint32_t m_readState;
};


template <typename T>
DoubleBuffer<T>::DoubleBuffer() : m_state(0) {}

template <typename T>
DoubleBuffer<T>::~DoubleBuffer() {}

template <typename T>
T* DoubleBuffer<T>::start_writing()
{
    // Increment active users; once we do this, no one
    // can swap the active cell on us until we are done.
    auto state = m_state.fetch_add(0x2, std::memory_order_relaxed);
    return &m_buf[state & 1];
}

template <typename T>
void DoubleBuffer<T>::end_writing()
{
    // We want to swap the active cell, but only if we were the last ones
    // concurrently accessing the data (otherwise the consumer will do it
    // for us when it is done accessing the data).
    auto state = m_state.load(std::memory_order_relaxed);
    std::uint32_t flag = (8 << (state & 1)) ^ (state & (8 << (state & 1)));
    state = m_state.fetch_add(flag - 0x2, std::memory_order_release) + flag - 0x2;
    if ((state & 0x6) == 0) {
        // The consumer is not reading, we should swap (unless the consumer has
        // since started a read or already swapped or read a value and is about
        // to swap). If we swap, we also want to clear the full flag on the new
        // active cell, otherwise the consumer could read two values out of
        // order (reads new value, swaps, reads old value while producer is idle).
        m_state.compare_exchange_strong(state,
                                        (state ^ 0x1) & ~(0x10 >> (state & 1)),
                                        std::memory_order_release);
    }
}

template <typename T>
T* DoubleBuffer<T>::start_reading()
{
    m_readState = m_state.load(std::memory_order_relaxed);

    // Return nullptr if there is nothing to read.
    if ((m_readState & (0x10 >> (m_readState & 1))) == 0) return nullptr;

    // At this point, we are sure there is something new to read, because the
    // full flag is never turned off by the producer once it is on; the only
    // thing that could happen is that the active cell changes, but this
    // happens only after the producer writes a value into it, in which case
    // there is still a value to read, just in a different cell.
    m_readState = m_state.fetch_add(0x2, std::memory_order_acquire) + 0x2;

    // Now that we have incremented user count, nobody can swap until we
    // decrement it.
    return &m_buf[(m_readState & 1) ^ 1];
}

template <typename T>
void DoubleBuffer<T>::end_reading()
{
    // Double check that there is nothing to read.
    if ((m_readState & (0x10 >> (m_readState & 1))) == 0) return;

    // At this point the active cell cannot change on us, but the active cell's
    // flag could change and also the users count could change. We want to
    // release our buffer (decrement user count) and remove the flag on the
    // value we read.
    auto state = m_state.load(std::memory_order_relaxed);
    std::uint32_t sub = (0x10 >> (state & 1)) | 0x2;
    state = m_state.fetch_sub(sub, std::memory_order_relaxed) - sub;
    if ((state & 0x6) == 0 && (state & (0x8 << (state & 1))) == 1) {
        // We were the last ones accessing the data when releasing out cell.
        // This means we should swap, but only if the producer is working and
        // has not swapped yet (and has not set the flag we just reset - that
        // would mean they swapped an even number of times). Note that we do
        // not bother swapping if there is nothing to read in the other cell.
        m_state.compare_exchange_strong(state,
                                        state ^ 0x1,
                                        std::memory_order_relaxed);
    }
}

#endif //DOUBLEBUFFER_HPP
