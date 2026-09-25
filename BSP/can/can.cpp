#include "can.h"
#include "canConfig.h"

template class CAN_<canConfig::classicCan>;  // 显式实例化 — classicCan
template class CAN_<canConfig::Fdcan>;       // 显式实例化 — Fdcan

// ——— HAL 中断回调 ———
extern "C" {
	/**
	 * @brief   HAL FDCAN FIFO0 接收中断回调
	 * @param   hfdcan      触发中断的 FDCAN 句柄
	 * @param   RxFifo0ITs  中断标志位掩码
	 * @note    根据 hfdcan->Init.FrameFormat 自动判断走 classicCan 还是 Fdcan
	 *          不在中断里做耗时操作
	 */
	void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
	{
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
			return;

		// 经典CAN
		if (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
			CAN_<canConfig::classicCan>::fifoxCallBack(hfdcan, FDCAN_RX_FIFO0);
		// FDCAN模式
		else
			CAN_<canConfig::Fdcan>::fifoxCallBack(hfdcan, FDCAN_RX_FIFO0);
	}

	/**
	 * @brief   HAL FDCAN FIFO1 接收中断回调
	 * @param   hfdcan      触发中断的 FDCAN 句柄
	 * @param   RxFifo1ITs  中断标志位掩码
	 * @note    逻辑与 FIFO0 回调一致，仅 fifox 参数不同
	 */
	void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
	{
		if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0)
			return;

		// 经典CAN
		if (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
			CAN_<canConfig::classicCan>::fifoxCallBack(hfdcan, FDCAN_RX_FIFO1);
		// FDCAN模式
		else
			CAN_<canConfig::Fdcan>::fifoxCallBack(hfdcan, FDCAN_RX_FIFO1);
	}

}