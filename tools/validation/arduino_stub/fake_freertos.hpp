#pragma once
#include "fake_board.hpp"
#include <cstring>

// Cooperative host scheduling only. Does not model FreeRTOS races or MCU timing.
using BaseType_t = int;
using TickType_t = uint32_t;
static const BaseType_t pdTRUE = 1;
static const BaseType_t pdPASS = 1;
static const TickType_t portMAX_DELAY = 0xFFFFFFFFu;
#define pdMS_TO_TICKS(value) (value)
struct ModelWouldBlock {};
struct ModelQueue
{
    unsigned capacity;
    unsigned width;
    std::deque<std::vector<uint8_t>> packets;
};
using QueueHandle_t = ModelQueue *;
struct ModelSemaphore { bool given = false; };
using SemaphoreHandle_t = ModelSemaphore *;
static void (*model_worker)(void *) = nullptr;
static bool force_queue_full = false;
static bool force_barrier_failure = false;
static int model_core = 1;
static int xPortGetCoreID() { return model_core; }
static QueueHandle_t xQueueCreate(unsigned capacity, unsigned width)
{
    return new ModelQueue{capacity, width, {}};
}
static SemaphoreHandle_t xSemaphoreCreateBinary() { return new ModelSemaphore; }
static BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore)
{
    semaphore->given = true;
    return pdTRUE;
}
static BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t)
{
    if (!semaphore->given || force_barrier_failure) { return 0; }
    semaphore->given = false;
    return pdTRUE;
}
static BaseType_t xTaskCreatePinnedToCore(void (*function)(void *), const char *, unsigned, void *, unsigned, void *, int core)
{
    assert(core == 0);
    model_worker = function;
    return pdPASS;
}
static BaseType_t xQueueReceive(QueueHandle_t queue, void *packet, TickType_t)
{
    if (queue->packets.empty()) { throw ModelWouldBlock{}; }
    std::memcpy(packet, queue->packets.front().data(), queue->width);
    queue->packets.pop_front();
    return pdTRUE;
}
static BaseType_t xQueueSend(QueueHandle_t queue, const void *packet, TickType_t)
{
    if (force_queue_full || queue->packets.size() >= queue->capacity) { return 0; }
    const uint8_t *bytes = (const uint8_t *)packet;
    queue->packets.push_back(std::vector<uint8_t>(bytes, bytes + queue->width));
    int previous_core = model_core;
    model_core = 0;
    try { model_worker(nullptr); }
    catch (const ModelWouldBlock &) {}
    model_core = previous_core;
    return pdTRUE;
}
static unsigned uxQueueMessagesWaiting(QueueHandle_t queue) { return (unsigned)queue->packets.size(); }
