/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */

/* TODO[CANFD-01/02 双板与硬件配置]
 * 上下板每路CAN用途不同，需按板卡角色集中定义模式、仲裁段/数据段时序、容量及过滤器数量，
 * 并同步CubeMX/.ioc或生成后配置流程；不能仅在应用层换句柄，外设仍使用另一块板的固定初始化。
 * DM和板间消息共线的那一路应支持真正FD帧（按实际选择BRS），保留其他DJI经典总线的原模式。
 * 当前FDCAN1/2是经典模式，FDCAN3是FD+BRS；这不是最终上下板接线表，不据此自动决定DM接哪路。
 *
 * TODO[CANFD-02 最大容量与Message RAM]
 * 共享路发送元素改为最大64字节；接收板间长帧的FIFO元素也须能容纳64，电机专用FIFO可按需保留8。
 * “元素容量64”不等于所有帧都发64：每个Tx帧DLC仍由实际8/64字节决定。
 * 若使用独立Rx Buffer，同样按可能命中的最长帧配置容量；软件接收数组也必须在启用长帧前完成扩容。
 * 三路FDCAN共用Message RAM，扩大元素后需重算所有区域及MessageRAMOffset，检查不重叠且总量不越界，
 * 不能只把某路DATA_BYTES_8替换成64就保留旧偏移。过滤器总数同时覆盖DM及板间各ID。
 * 两块板与同路所有FD设备的波特率/BRS需匹配，物理收发器也必须支持选定的数据段速率。
 */

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{
	/* USER CODE BEGIN FDCAN1_Init 0 */

	/* USER CODE END FDCAN1_Init 0 */

	/* USER CODE BEGIN FDCAN1_Init 1 */
	/* TODO[CANFD-02 FDCAN1]：若本板角色把此路用于DM+板间，联动下方FrameFormat、时序、
	 * TxElmtSize及接收长帧FIFO容量，并重新核对三路Message RAM布局；若仍为DJI经典路则不升级。
	 * 本段位于HAL_FDCAN_Init前，可用于板卡配置策略；须与工程生成配置保持一致。 */

	/* USER CODE END FDCAN1_Init 1 */
	hfdcan1.Instance                  = FDCAN1;
	hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission   = DISABLE;
	hfdcan1.Init.TransmitPause        = DISABLE;
	hfdcan1.Init.ProtocolException    = ENABLE;
	hfdcan1.Init.NominalPrescaler     = 1;
	hfdcan1.Init.NominalSyncJumpWidth = 20;
	hfdcan1.Init.NominalTimeSeg1      = 59;
	hfdcan1.Init.NominalTimeSeg2      = 20;
	hfdcan1.Init.DataPrescaler        = 1;
	hfdcan1.Init.DataSyncJumpWidth    = 2;
	hfdcan1.Init.DataTimeSeg1         = 13;
	hfdcan1.Init.DataTimeSeg2         = 2;
	hfdcan1.Init.MessageRAMOffset     = 0;
	hfdcan1.Init.StdFiltersNbr        = 8;
	hfdcan1.Init.ExtFiltersNbr        = 4;
	hfdcan1.Init.RxFifo0ElmtsNbr      = 8;
	hfdcan1.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.RxFifo1ElmtsNbr      = 8;
	hfdcan1.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.RxBuffersNbr         = 2;
	hfdcan1.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
	hfdcan1.Init.TxEventsNbr          = 8;
	hfdcan1.Init.TxBuffersNbr         = 8;
	hfdcan1.Init.TxFifoQueueElmtsNbr  = 8;
	hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
	hfdcan1.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN1_Init 2 */

	/* USER CODE END FDCAN1_Init 2 */
}

/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{
	/* USER CODE BEGIN FDCAN2_Init 0 */

	/* USER CODE END FDCAN2_Init 0 */

	/* USER CODE BEGIN FDCAN2_Init 1 */
	/* TODO[CANFD-02 FDCAN2]：按本板实际总线用途决定是否升级FD及64字节，不与FDCAN1强行相同。
	 * 改容量时核对下方MessageRAMOffset和后续FDCAN3起始位置，不能覆盖相邻控制器的RAM区域。 */

	/* USER CODE END FDCAN2_Init 1 */
	hfdcan2.Instance                  = FDCAN2;
	hfdcan2.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
	hfdcan2.Init.Mode                 = FDCAN_MODE_NORMAL;
	hfdcan2.Init.AutoRetransmission   = ENABLE;
	hfdcan2.Init.TransmitPause        = DISABLE;
	hfdcan2.Init.ProtocolException    = ENABLE;
	hfdcan2.Init.NominalPrescaler     = 1;
	hfdcan2.Init.NominalSyncJumpWidth = 20;
	hfdcan2.Init.NominalTimeSeg1      = 59;
	hfdcan2.Init.NominalTimeSeg2      = 20;
	hfdcan2.Init.DataPrescaler        = 1;
	hfdcan2.Init.DataSyncJumpWidth    = 2;
	hfdcan2.Init.DataTimeSeg1         = 13;
	hfdcan2.Init.DataTimeSeg2         = 2;
	hfdcan2.Init.MessageRAMOffset     = 0x406;
	hfdcan2.Init.StdFiltersNbr        = 4;
	hfdcan2.Init.ExtFiltersNbr        = 4;
	hfdcan2.Init.RxFifo0ElmtsNbr      = 8;
	hfdcan2.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxFifo1ElmtsNbr      = 8;
	hfdcan2.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxBuffersNbr         = 3;
	hfdcan2.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.TxEventsNbr          = 8;
	hfdcan2.Init.TxBuffersNbr         = 8;
	hfdcan2.Init.TxFifoQueueElmtsNbr  = 8;
	hfdcan2.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
	hfdcan2.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN2_Init 2 */

	/* USER CODE END FDCAN2_Init 2 */
}

/* FDCAN3 init function */
void MX_FDCAN3_Init(void)
{
	/* USER CODE BEGIN FDCAN3_Init 0 */

	/* USER CODE END FDCAN3_Init 0 */

	/* USER CODE BEGIN FDCAN3_Init 1 */
	/* TODO[CANFD-02 FDCAN3]：此路已有FD+BRS，但下方Tx/Rx元素仍为8字节；板间长帧尚不能直接启用。
	 * 若共线DM+板间，按上述总说明扩大所需元素并重算RAM布局；最终时序以两端设备配置为准。 */

	/* USER CODE END FDCAN3_Init 1 */
	hfdcan3.Instance                  = FDCAN3;
	hfdcan3.Init.FrameFormat          = FDCAN_FRAME_FD_BRS;
	hfdcan3.Init.Mode                 = FDCAN_MODE_NORMAL;
	hfdcan3.Init.AutoRetransmission   = DISABLE;
	hfdcan3.Init.TransmitPause        = DISABLE;
	hfdcan3.Init.ProtocolException    = DISABLE;
	hfdcan3.Init.NominalPrescaler     = 1;
	hfdcan3.Init.NominalSyncJumpWidth = 20;
	hfdcan3.Init.NominalTimeSeg1      = 59;
	hfdcan3.Init.NominalTimeSeg2      = 20;
	hfdcan3.Init.DataPrescaler        = 1;
	hfdcan3.Init.DataSyncJumpWidth    = 2;
	hfdcan3.Init.DataTimeSeg1         = 13;
	hfdcan3.Init.DataTimeSeg2         = 2;
	hfdcan3.Init.MessageRAMOffset     = 0x812;
	hfdcan3.Init.StdFiltersNbr        = 4;
	hfdcan3.Init.ExtFiltersNbr        = 4;
	hfdcan3.Init.RxFifo0ElmtsNbr      = 8;
	hfdcan3.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
	hfdcan3.Init.RxFifo1ElmtsNbr      = 8;
	hfdcan3.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
	hfdcan3.Init.RxBuffersNbr         = 2;
	hfdcan3.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
	hfdcan3.Init.TxEventsNbr          = 8;
	hfdcan3.Init.TxBuffersNbr         = 8;
	hfdcan3.Init.TxFifoQueueElmtsNbr  = 8;
	hfdcan3.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
	hfdcan3.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN3_Init 2 */

	/* USER CODE END FDCAN3_Init 2 */
}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
	GPIO_InitTypeDef         GPIO_InitStruct     = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	if (fdcanHandle->Instance == FDCAN1)
	{
		/* USER CODE BEGIN FDCAN1_MspInit 0 */

		/* USER CODE END FDCAN1_MspInit 0 */

		/** Initializes the peripherals clock
  */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
		PeriphClkInitStruct.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		{
			Error_Handler();
		}

		/* FDCAN1 clock enable */
		HAL_RCC_FDCAN_CLK_ENABLED++;
		if (HAL_RCC_FDCAN_CLK_ENABLED == 1)
		{
			__HAL_RCC_FDCAN_CLK_ENABLE();
		}

		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
		GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* FDCAN1 interrupt Init */
		HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
		HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
		/* USER CODE BEGIN FDCAN1_MspInit 1 */

		/* USER CODE END FDCAN1_MspInit 1 */
	} else if (fdcanHandle->Instance == FDCAN2)
	{
		/* USER CODE BEGIN FDCAN2_MspInit 0 */

		/* USER CODE END FDCAN2_MspInit 0 */

		/** Initializes the peripherals clock
  */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
		PeriphClkInitStruct.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		{
			Error_Handler();
		}

		/* FDCAN2 clock enable */
		HAL_RCC_FDCAN_CLK_ENABLED++;
		if (HAL_RCC_FDCAN_CLK_ENABLED == 1)
		{
			__HAL_RCC_FDCAN_CLK_ENABLE();
		}

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
		GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* FDCAN2 interrupt Init */
		HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
		HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
		/* USER CODE BEGIN FDCAN2_MspInit 1 */

		/* USER CODE END FDCAN2_MspInit 1 */
	} else if (fdcanHandle->Instance == FDCAN3)
	{
		/* USER CODE BEGIN FDCAN3_MspInit 0 */

		/* USER CODE END FDCAN3_MspInit 0 */

		/** Initializes the peripherals clock
  */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
		PeriphClkInitStruct.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		{
			Error_Handler();
		}

		/* FDCAN3 clock enable */
		HAL_RCC_FDCAN_CLK_ENABLED++;
		if (HAL_RCC_FDCAN_CLK_ENABLED == 1)
		{
			__HAL_RCC_FDCAN_CLK_ENABLE();
		}

		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**FDCAN3 GPIO Configuration
    PD12     ------> FDCAN3_RX
    PD13     ------> FDCAN3_TX
    */
		GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF5_FDCAN3;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* FDCAN3 interrupt Init */
		HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
		HAL_NVIC_SetPriority(FDCAN3_IT1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(FDCAN3_IT1_IRQn);
		/* USER CODE BEGIN FDCAN3_MspInit 1 */

		/* USER CODE END FDCAN3_MspInit 1 */
	}
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{
	if (fdcanHandle->Instance == FDCAN1)
	{
		/* USER CODE BEGIN FDCAN1_MspDeInit 0 */

		/* USER CODE END FDCAN1_MspDeInit 0 */
		/* Peripheral clock disable */
		HAL_RCC_FDCAN_CLK_ENABLED--;
		if (HAL_RCC_FDCAN_CLK_ENABLED == 0)
		{
			__HAL_RCC_FDCAN_CLK_DISABLE();
		}

		/**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);

		/* FDCAN1 interrupt Deinit */
		HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
		HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
		/* USER CODE BEGIN FDCAN1_MspDeInit 1 */

		/* USER CODE END FDCAN1_MspDeInit 1 */
	} else if (fdcanHandle->Instance == FDCAN2)
	{
		/* USER CODE BEGIN FDCAN2_MspDeInit 0 */

		/* USER CODE END FDCAN2_MspDeInit 0 */
		/* Peripheral clock disable */
		HAL_RCC_FDCAN_CLK_ENABLED--;
		if (HAL_RCC_FDCAN_CLK_ENABLED == 0)
		{
			__HAL_RCC_FDCAN_CLK_DISABLE();
		}

		/**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

		/* FDCAN2 interrupt Deinit */
		HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
		HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
		/* USER CODE BEGIN FDCAN2_MspDeInit 1 */

		/* USER CODE END FDCAN2_MspDeInit 1 */
	} else if (fdcanHandle->Instance == FDCAN3)
	{
		/* USER CODE BEGIN FDCAN3_MspDeInit 0 */

		/* USER CODE END FDCAN3_MspDeInit 0 */
		/* Peripheral clock disable */
		HAL_RCC_FDCAN_CLK_ENABLED--;
		if (HAL_RCC_FDCAN_CLK_ENABLED == 0)
		{
			__HAL_RCC_FDCAN_CLK_DISABLE();
		}

		/**FDCAN3 GPIO Configuration
    PD12     ------> FDCAN3_RX
    PD13     ------> FDCAN3_TX
    */
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13);

		/* FDCAN3 interrupt Deinit */
		HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
		HAL_NVIC_DisableIRQ(FDCAN3_IT1_IRQn);
		/* USER CODE BEGIN FDCAN3_MspDeInit 1 */

		/* USER CODE END FDCAN3_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

