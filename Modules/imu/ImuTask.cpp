/**
 * @file ImuTask.cpp
 * @brief 在任务中读取 BMI088 并发布六轴融合后的姿态。
 */

#include "ImuTask.h"

#include "Bmi088.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include "task.h"

namespace imu
{
namespace
{
    osThreadId_t taskHandle = nullptr;
    AttitudeSample latest{};
    AttitudeEstimator estimator{};

    constexpr osThreadAttr_t taskAttrs = {
        .name = "ImuTask",
        .stack_size = 3072U,
        .priority = osPriorityAboveNormal,
    };

    /** @brief 用短临界区发布一份完整姿态，避免 CDC 读到混合帧。 */
    void Publish(const AttitudeSample& sample)
    {
        taskENTER_CRITICAL();
        latest = sample;
        taskEXIT_CRITICAL();
    }

    /** @brief 初始化后约 500 Hz 采样；SPI 失败时撤销上一帧有效性。 */
    void Task(void*)
    {
        const uint32_t configuredFrequency = osKernelGetTickFreq();
        const uint32_t tickFrequency = configuredFrequency == 0U ? 1U : configuredFrequency;
        const uint32_t sampleTicks = tickFrequency >= 500U ? tickFrequency / 500U : 1U;

        while (BMI088_ConfigureMotion() != HAL_OK)
        {
            Publish({});
            osDelay(tickFrequency);
        }

        uint32_t previousTick = 0U;
        for (;;)
        {
            Bmi088MotionSample motion{};
            if (BMI088_ReadMotion(&motion) != HAL_OK)
            {
                estimator.Reset();
                previousTick = 0U;
                Publish({});
            }
            else
            {
                const uint32_t now = osKernelGetTickCount();
                const float dt_s = previousTick == 0U ? 0.0F :
                    static_cast<float>(now - previousTick) / static_cast<float>(tickFrequency);
                previousTick = now;
                AttitudeSample sample = estimator.Update(motion.acceleration_mps2,
                                                           motion.angular_velocity_rad_s, dt_s);
                sample.tick = now;
                Publish(sample);
            }
            osDelay(sampleTicks);
        }
    }
} // namespace

void CreateTask()
{
    if (taskHandle != nullptr)
        return;
    taskHandle = osThreadNew(Task, nullptr, &taskAttrs);
    if (taskHandle == nullptr)
        Error_Handler();
}

bool Latest(AttitudeSample* output)
{
    if (output == nullptr)
        return false;
    taskENTER_CRITICAL();
    *output = latest;
    taskEXIT_CRITICAL();
    const uint32_t freshnessTicks = osKernelGetTickFreq() / 10U;
    return output->valid &&
           osKernelGetTickCount() - output->tick <= freshnessTicks;
}
} // namespace imu
