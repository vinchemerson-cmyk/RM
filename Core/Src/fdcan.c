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

#include "board_can_config.h"

/* 三路共享Message RAM，按实际8/64字节元素连续分配；上下板配置来自C/C++共用头。 */
static void ApplyBoardCanConfiguration(FDCAN_HandleTypeDef* handle, unsigned index)
{
    const BoardCanConfig* config = &kBoardCanConfig[index];
    if (config->mode > BOARD_CAN_FD_BRS || config->standard_filters == 0U ||
        config->standard_filters > 128U || BoardCanTotalRamWords() >= 2560U ||
        !IS_FDCAN_NOMINAL_PRESCALER(config->nominal_prescaler) ||
        !IS_FDCAN_NOMINAL_SJW(config->nominal_sjw) ||
        !IS_FDCAN_NOMINAL_TSEG1(config->nominal_seg1) ||
        !IS_FDCAN_NOMINAL_TSEG2(config->nominal_seg2) ||
        !IS_FDCAN_DATA_PRESCALER(config->data_prescaler) ||
        !IS_FDCAN_DATA_SJW(config->data_sjw) ||
        !IS_FDCAN_DATA_TSEG1(config->data_seg1) ||
        !IS_FDCAN_DATA_TSEG2(config->data_seg2))
        Error_Handler();
    handle->Init.FrameFormat = config->mode == BOARD_CAN_CLASSIC ? FDCAN_FRAME_CLASSIC :
                              config->mode == BOARD_CAN_FD ? FDCAN_FRAME_FD_NO_BRS : FDCAN_FRAME_FD_BRS;
    handle->Init.NominalPrescaler = config->nominal_prescaler;
    handle->Init.NominalSyncJumpWidth = config->nominal_sjw;
    handle->Init.NominalTimeSeg1 = config->nominal_seg1;
    handle->Init.NominalTimeSeg2 = config->nominal_seg2;
    handle->Init.DataPrescaler = config->data_prescaler;
    handle->Init.DataSyncJumpWidth = config->data_sjw;
    handle->Init.DataTimeSeg1 = config->data_seg1;
    handle->Init.DataTimeSeg2 = config->data_seg2;
    const uint32_t size = config->mode == BOARD_CAN_CLASSIC ? FDCAN_DATA_BYTES_8 : FDCAN_DATA_BYTES_64;
    handle->Init.StdFiltersNbr = config->standard_filters;
    handle->Init.ExtFiltersNbr = 4U;
    handle->Init.RxFifo0ElmtsNbr = 8U;
    handle->Init.RxFifo1ElmtsNbr = 8U;
    handle->Init.RxBuffersNbr = index == 1U ? 3U : 2U;
    handle->Init.TxEventsNbr = 8U;
    handle->Init.TxBuffersNbr = 8U;
    handle->Init.TxFifoQueueElmtsNbr = 8U;
    handle->Init.RxFifo0ElmtSize = size;
    handle->Init.RxFifo1ElmtSize = size;
    handle->Init.RxBufferSize = size;
    handle->Init.TxElmtSize = size;
    handle->Init.MessageRAMOffset = BoardCanRamOffset(index);
}




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
	/* Generated defaults above are overridden by the selected board profile here. */
	ApplyBoardCanConfiguration(&hfdcan1, 0U);
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
	/* Generated defaults above are overridden by the selected board profile here. */
	ApplyBoardCanConfiguration(&hfdcan2, 1U);
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
	/* Generated defaults above are overridden by the selected board profile here. */
	ApplyBoardCanConfiguration(&hfdcan3, 2U);
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

