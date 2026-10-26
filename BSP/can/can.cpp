/**
 * @file can.cpp
 * @brief 分发专用 FDCAN 接收回调，并实现 CAN 设备接口。
 */

#include "can.h"
#include "canConfig.h"

namespace
{
	FDCAN_HandleTypeDef*  dedicated_handle  = nullptr;
	CanDedicatedRxHandler dedicated_handler = nullptr;

	/** @brief 处理进入回调时已排队的帧，HAL 读取失败也不会在中断中无限循环。 */
	template <typename Traits>
	void DrainFifo(FDCAN_HandleTypeDef* handle, uint32_t fifo)
	{
		uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(handle, fifo);
		while (pending-- != 0U)
			CAN_<Traits>::fifoxCallBack(handle, fifo);
	}
}

/** @brief 保存独占 FDCAN 控制器及其接收处理回调。 */
void CAN_SetDedicatedRxHandler(FDCAN_HandleTypeDef* handle, CanDedicatedRxHandler handler)
{
	dedicated_handle  = handle;
	dedicated_handler = handler;
}

/** @brief 优先把匹配的 FIFO 事件交给独占模块。 */
bool CAN_DispatchDedicatedRx(FDCAN_HandleTypeDef* handle, uint32_t fifo)
{
	if (handle != dedicated_handle || dedicated_handler == nullptr)
		return false;
	dedicated_handler(handle, fifo);
	return true;
}

template class CAN_<canConfig::classicCan>; // 显式实例化 — classicCan
template class CAN_<canConfig::Fdcan>;      // 显式实例化 — Fdcan

// ——— HAL 中断回调 ———
extern "C" {
	/**
	 * @brief   HAL FDCAN FIFO0 接收中断回调
	 * @param   hfdcan      触发中断的 FDCAN 句柄
	 * @param   RxFifo0ITs  中断标志位掩码
	 * @note    根据 hfdcan->Init.FrameFormat 自动判断走 classicCan 还是 Fdcan
	 *          不在中断里做耗时操作
	 */
	void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
	{
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
			return;
		// TODO[CANFD-05 取消整路独占]：共享DM与板间消息时，不可让DedicatedRx提前吃掉整路FIFO。
		// 应使用统一接收注册表分发给各模块，或先实现严格的按FIFO专用路由；不可让两条路径重复取帧。
		if (CAN_DispatchDedicatedRx(hfdcan, FDCAN_RX_FIFO0))
			return;

		// TODO[CANFD-01 后端一致]：FD模式的本路DM和板间实例统一注册Fdcan表，classic模式统一注册classicCan表。
		// 同一FD表里的tx_len/rx_len可以分别为8或64，无需按帧长切换模板或调用两个DrainFifo。
		// 经典CAN
		if (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
			DrainFifo<canConfig::classicCan>(hfdcan, FDCAN_RX_FIFO0);
			// FDCAN模式
		else
			DrainFifo<canConfig::Fdcan>(hfdcan, FDCAN_RX_FIFO0);
	}

	/**
	 * @brief   HAL FDCAN FIFO1 接收中断回调
	 * @param   hfdcan      触发中断的 FDCAN 句柄
	 * @param   RxFifo1ITs  中断标志位掩码
	 * @note    逻辑与 FIFO0 回调一致，仅 fifox 参数不同
	 */
	void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
	{
		if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0)
			return;
		// TODO[CANFD-05 FIFO1]：若将板间消息路由到FIFO1，必须同步调整DedicatedRx的拦截范围，
		// 目前DedicatedRx按整路句柄判断，不能以为换FIFO后自然就不影响电机反馈。
		if (CAN_DispatchDedicatedRx(hfdcan, FDCAN_RX_FIFO1))
			return;

		// 经典CAN
		if (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
			DrainFifo<canConfig::classicCan>(hfdcan, FDCAN_RX_FIFO1);
			// FDCAN模式
		else
			DrainFifo<canConfig::Fdcan>(hfdcan, FDCAN_RX_FIFO1);
	}
}
