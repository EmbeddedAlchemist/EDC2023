#pragma once

#include <stddef.h>
#include <stdint.h>

// #include "../Source/FreeRTOS/FreeRTOSSupport.hpp"
#include "ExLib_Units.hpp"

namespace ExLib {

enum class Task_State {
    Running = 0,
    Ready,
    Blocked,
    Suspended,
    Deleted,
    Invalid
};

class Task {
  private:
    void *taskHandler;
    void (*const taskFunction)(void *);
    void *const taskParam;
    const char *const taskName;
    const std::uint32_t priority;
    const std::size_t stackDepth;

  public:
    Task() = delete;
    Task(void (*taskFunction)(void *), void *param = nullptr, const char *taskName = "Default", std::uint32_t priority = 1, std::size_t stackDepth = 256);
    ~Task();
    bool begin(void);
    void end(void);
    void suspend(void);
    void resume(void);
    Task_State getState();

    static void deleteCurrent();
};

//
//
//

template <typename Type>
class Queue {
    void *handler;

  public:
    Queue(std::size_t length);
    void sendWithBlocking(Type &data);
    bool sendWithoutBlocking(Type &data);
    bool sendWithTimeout(Type &data, TimeInterval maxDelay);

    void receiveWithBlocking(Type &data);
    bool receiveWithoutBlocking(Type &data);
    bool receiveWithTimeout(Type &data, TimeInterval maxDelay);

    std::size_t available(void);
    std::size_t remainSpace(void);
};

template <typename Type>
inline Queue<Type>::Queue(std::size_t length) {
    extern void *queueCreate(std::size_t length, std::size_t unitSize);
    handler = queueCreate(length, sizeof(Type));
}

template <typename Type>
inline void Queue<Type>::sendWithBlocking(Type &data) {
    extern void queueSendBlocking(void *handler, void *data);
    queueSendBlocking(handler, &data);
}

template <typename Type>
inline bool Queue<Type>::sendWithoutBlocking(Type &data) {
    return sendWithTimeout(data, TimeInterval(0));
}

template <typename Type>
inline bool Queue<Type>::sendWithTimeout(Type &data, TimeInterval maxDelay) {
    extern bool queueSendWithTimeout(void *handler, void *data, TimeInterval timeout);
    return queueSendWithTimeout(handler, &data, maxDelay);
}

template <typename Type>
inline void Queue<Type>::receiveWithBlocking(Type &data) {
    extern void queueReceiveBlocking(void *handler, void *data);
    queueReceiveBlocking(handler, &data);
}

template <typename Type>
inline bool Queue<Type>::receiveWithoutBlocking(Type &data) {
    return receiveWithTimeout(data, TimeInterval(0));
}

template <typename Type>
inline bool Queue<Type>::receiveWithTimeout(Type &data, TimeInterval maxDelay) {
    extern void queueReceiveWithTimeout(void *handler, void *data, TimeInterval timeout);
    return queueReceiveWithTimeout(handler, &data, maxDelay);
}

template <typename Type>
inline std::size_t Queue<Type>::available(void) {
    std::size_t queueGetAvailable(void *handler);
    return queueGetAvailable(handler);
}

template <typename Type>
inline std::size_t Queue<Type>::remainSpace(void) {
    std::size_t queueGetRemain(void *handler);
    return queueGetRemain(handler);
}

} // namespace ExLib