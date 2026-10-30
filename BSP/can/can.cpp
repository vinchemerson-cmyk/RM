/** @file can.cpp @brief 统一读取共享FIFO，按物理总线模式选择唯一注册后端。 */
#include "can.h"

namespace
{
	/**
	 * @brief 中断排空fifo
	 * @param handle
	 * @param fifo
	 */
	void DrainFifo(FDCAN_HandleTypeDef* handle, uint32_t fifo)
    {
        // 获取当前排队的帧数
        uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(handle, fifo);

		//倒计数循环.假设 pending 进来时是 3，循环体会执行 3 次，直到 pending 递减到 0 退出
		//这个循环的次数是固定的，只处理进入中断那一刻已经存在的帧
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
