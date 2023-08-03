#include "ExLib_FreeRTOS.hpp"
#include "FreeRTOS/FreeRTOSSupport.hpp"

namespace ExLib {
Task::Task(void (*taskFunction)(void *), void *param, const char *taskName, std::uint32_t priority, std::size_t stackDepth)
    : taskFunction(taskFunction),
      taskParam(param),
      taskName(taskName),
      priority(priority),
      stackDepth(stackDepth),
      taskHandler(nullptr) {
}

Task::~Task() {
    end();
}

bool Task::begin() {
    BaseType_t result;
    result = xTaskCreate(taskFunction, taskName, stackDepth, taskParam, priority, (TaskHandle_t *)&taskHandler);
    if (result != pdPASS) {
        vTaskDelete((TaskHandle_t)taskHandler);
        taskHandler = nullptr;
        return false;
    }
    return true;
}

void Task::end() {
    vTaskDelete((TaskHandle_t)taskHandler);
    taskHandler = nullptr;
}

void Task::suspend() {
    vTaskSuspend((TaskHandle_t)taskHandler);
}

void Task::resume() {
    vTaskResume((TaskHandle_t)taskHandler);
}

Task_State Task::getState() {
    eTaskState state = eTaskGetState((TaskHandle_t)taskHandler);
    return (Task_State)state;
}

void Task::deleteCurrent() {
    vTaskDelete(NULL);
}

void *queueCreate(std::size_t length, std::size_t unitSize) {
    return xQueueCreate(length, unitSize);
}

void queueSendBlocking(void *handler, void *data) {
    xQueueSend((QueueHandle_t)handler, data, portMAX_DELAY);
}

bool queueSendWithTimeout(void *handler, void *data, TimeInterval timeout) {
    return xQueueSend((QueueHandle_t)handler, data, timeout.us / 1000 / portTICK_PERIOD_MS) == pdPASS;
}

void queueReceiveBlocking(void *handler, void *data) {
    xQueueReceive((QueueHandle_t)handler, data, portMAX_DELAY);
}

void queueReceiveWithTimeout(void *handler, void *data, TimeInterval timeout) {
    xQueueReceive((QueueHandle_t)handler, data, timeout.us / 1000 / portTICK_PERIOD_MS);
}

std::size_t queueGetAvailable(void *handler){
    return uxQueueMessagesWaiting((QueueHandle_t)handler);
}

std::size_t queueGetRemain(void *handler){
    return uxQueueSpacesAvailable((QueueHandle_t)handler);
}

bool queueSendFromISR(void *handler, void *data){
    return xQueueSendFromISR((QueueHandle_t)handler, data, nullptr);
}

} // namespace ExLib