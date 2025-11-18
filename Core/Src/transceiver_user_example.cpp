#include "jc24btran.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h" // expect UART handle extern (e.g., huart1)
#include <cstdint>

// 示例：在 FreeRTOS 中创建一个任务初始化并周期性处理 DataTransceiver，
// 并在任务内安全读取去抖后的 positions。

static jc24b::DataTransceiver g_transceiver;

// 队列：用于在通信任务和电机控制任务之间传递编码后的 positions（1 字节 0..15）
static QueueHandle_t xPositionsQueue = nullptr;

// 将 4-bit positions 编码为 0..15
static inline uint8_t encode_positions(const bool pos[4]) {
    return (pos[0] ? 1u : 0u) | (pos[1] ? 2u : 0u) | (pos[2] ? 4u : 0u) | (pos[3] ? 8u : 0u);
}

// 用户需实现或替换此函数以实际驱动电机（更新 PWM 占空比）。
// duty 的范围及单位由用户实现决定（例如 0..MAX_DUTY 或 0..1000）。
extern "C" void SetMotorDuty(uint32_t duty);

// MotorTask：从队列读取编码后的等级并映射为占空比，做斜率限制后调用 SetMotorDuty
extern "C" void MotorTask(void* pvParameters) {
    uint8_t recv = 0;
    uint32_t currentDuty = 0;
    const TickType_t period = pdMS_TO_TICKS(20); // 控制周期 20 ms

    // 根据你的 PWM 实现调整以下常量
    const uint32_t MAX_DUTY = 1000; // 例如定时器的 ARR 值或 1000 表示 100%
    const uint32_t maxStep = MAX_DUTY / 20; // 每周期最大变化量（防止突变）

    for (;;) {
        // 等待最新目标（有超时可在超时中实现安全降速）
        if (xQueueReceive(xPositionsQueue, &recv, period) == pdPASS) {
            uint8_t level = recv & 0x0F; // 0..15
            uint32_t targetDuty = (static_cast<uint32_t>(level) * MAX_DUTY) / 15u;

            // 斜率限制
            if (targetDuty > currentDuty) {
                uint32_t delta = targetDuty - currentDuty;
                if (delta > maxStep) delta = maxStep;
                currentDuty += delta;
            } else {
                uint32_t delta = currentDuty - targetDuty;
                if (delta > maxStep) delta = maxStep;
                currentDuty -= delta;
            }

            // 更新电机驱动
            SetMotorDuty(currentDuty);
        } else {
            // 超时未接收新值：可选择缓慢减速或保持当前值
            // 示例：缓慢减速到 0
            if (currentDuty > maxStep) currentDuty -= maxStep;
            else currentDuty = 0;
            SetMotorDuty(currentDuty);
        }
    }
}

extern "C" void TransceiverTask(void* params) {
    // 建议周期：100 ms
    const TickType_t delay = pdMS_TO_TICKS(100);
    bool lastPos[4] = {false, false, false, false};

    for (;;) {
        // 周期性处理（看门狗、重连）
        g_transceiver.process();

        // 安全读取 positions：使用 FreeRTOS 临界区以避免与中断回调竞态
        bool pos[4] = {false, false, false, false};
        taskENTER_CRITICAL();
        bool ok = g_transceiver.getPositions(pos);
        taskEXIT_CRITICAL();

        if (ok) {
            bool changed = false;
            for (int i = 0; i < 4; ++i) {
                if (pos[i] != lastPos[i]) {
                    changed = true;
                    lastPos[i] = pos[i];
                }
            }
            if (changed) {
                // 把编码后的 positions 发送到电机控制队列（非阻塞）
                if (xPositionsQueue != nullptr) {
                    uint8_t code = encode_positions(pos);
                    xQueueSend(xPositionsQueue, &code, 0);
                }
            }
        } else {
            // 数据无效或未连接；可做降级处理
        }

        vTaskDelay(delay);
    }
}

// 在系统初始化完成后调用该函数以启动 transceiver
// 注意：huart1 需要在此调用前被初始化
void StartTransceiverInterface() {
    // 假设在项目中存在 UART_HandleTypeDef huart1，并在 main.h 中声明为 extern
    extern UART_HandleTypeDef huart1; // main.c 中定义

    g_transceiver.init(&huart1);

    // 创建队列（8 个元素，每个元素 1 字节）
    if (xPositionsQueue == nullptr) {
        xPositionsQueue = xQueueCreate(8, sizeof(uint8_t));
    }

    // 创建任务（根据工程调整栈大小与优先级）
    xTaskCreate(TransceiverTask, "Transcv", 512, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    xTaskCreate(MotorTask, "Motor", 512, nullptr, tskIDLE_PRIORITY + 3, nullptr);
}
