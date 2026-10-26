/** @file can.cpp @brief 统一读取共享FIFO，按物理总线模式选择唯一注册后端。 */
#include "can.h"

namespace
{
    void DrainFifo(FDCAN_HandleTypeDef* handle, uint32_t fifo)
    {
        // 仅处理进入中断时已经排队的帧，限制ISR工作量，避免持续来帧导致中断不退出。
        uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(handle, fifo);
        while (pending-- != 0U)
        {
            if (handle->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
                CAN_<canConfig::classicCan>::fifoxCallBack(handle, fifo);
            else
                CAN_<canConfig::Fdcan>::fifoxCallBack(handle, fifo);
        }
    }
}
template class CAN_<canConfig::classicCan>;
template class CAN_<canConfig::Fdcan>;

extern "C" {
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t events)
{
    if ((events & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) DrainFifo(handle, FDCAN_RX_FIFO0);
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* handle, uint32_t events)
{
    if ((events & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U) DrainFifo(handle, FDCAN_RX_FIFO1);
}
}
