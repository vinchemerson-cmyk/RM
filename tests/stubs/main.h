/** @file main.h @brief 宿主机 CAN HAL 替身，仅由 tests/CMakeLists.txt 使用。 */
#pragma once
#include <cstdint>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY };

enum HAL_FDCAN_StateTypeDef { HAL_FDCAN_STATE_READY, HAL_FDCAN_STATE_BUSY };

struct FDCAN_HandleTypeDef
{
	struct
	{
		uint32_t StdFiltersNbr{16U};
		uint32_t FrameFormat{};
	} Init;

	HAL_FDCAN_StateTypeDef State{HAL_FDCAN_STATE_READY};
};

struct FDCAN_TxHeaderTypeDef
{
	uint32_t Identifier{},    IdType{},   TxFrameType{},        DataLength{}, ErrorStateIndicator{};
	uint32_t BitRateSwitch{}, FDFormat{}, TxEventFifoControl{}, MessageMarker{};
};

struct FDCAN_RxHeaderTypeDef
{
	uint32_t FilterIndex{}, DataLength{};
};

struct FDCAN_FilterTypeDef
{
	uint32_t IdType{}, FilterIndex{}, FilterType{}, FilterConfig{}, FilterID1{}, FilterID2{};
};

inline constexpr uint32_t FDCAN_FRAME_CLASSIC = 0U;
inline constexpr uint32_t FDCAN_FRAME_FD_BRS = 1U;
inline constexpr uint32_t FDCAN_CLASSIC_CAN = 0U;
inline constexpr uint32_t FDCAN_RX_FIFO0 = 0U;
inline constexpr uint32_t FDCAN_RX_FIFO1 = 1U;
inline constexpr uint32_t FDCAN_IT_RX_FIFO0_NEW_MESSAGE = 1U;
inline constexpr uint32_t FDCAN_IT_RX_FIFO1_NEW_MESSAGE = 16U;
inline constexpr uint32_t FDCAN_FILTER_TO_RXFIFO0 = 1U;
inline constexpr uint32_t FDCAN_FILTER_TO_RXFIFO1 = 2U;
inline constexpr uint32_t FDCAN_FILTER_MASK = 2U;
inline constexpr uint32_t FDCAN_STANDARD_ID = 0U;
inline constexpr uint32_t FDCAN_REJECT = 2U;
inline constexpr uint32_t FDCAN_REJECT_REMOTE = 1U;
inline constexpr uint32_t FDCAN_DATA_FRAME = 0U;
inline constexpr uint32_t FDCAN_ESI_ACTIVE = 0U;
inline constexpr uint32_t FDCAN_BRS_OFF = 0U;
inline constexpr uint32_t FDCAN_NO_TX_EVENTS = 0U;
inline constexpr uint32_t FDCAN_DLC_BYTES_0 = 0U, FDCAN_DLC_BYTES_1 = 1U;
inline constexpr uint32_t FDCAN_DLC_BYTES_2 = 2U, FDCAN_DLC_BYTES_3 = 3U;
inline constexpr uint32_t FDCAN_DLC_BYTES_4 = 4U, FDCAN_DLC_BYTES_5 = 5U;
inline constexpr uint32_t FDCAN_DLC_BYTES_6 = 6U, FDCAN_DLC_BYTES_7 = 7U;
inline constexpr uint32_t FDCAN_DLC_BYTES_8 = 8U, FDCAN_DLC_BYTES_12 = 9U;
inline constexpr uint32_t FDCAN_DLC_BYTES_16 = 10U, FDCAN_DLC_BYTES_20 = 11U;
inline constexpr uint32_t FDCAN_DLC_BYTES_24 = 12U, FDCAN_DLC_BYTES_32 = 13U;
inline constexpr uint32_t FDCAN_DLC_BYTES_48 = 14U, FDCAN_DLC_BYTES_64 = 15U;
void Error_Handler();
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, const FDCAN_FilterTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
HAL_FDCAN_StateTypeDef HAL_FDCAN_GetState(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*, const FDCAN_TxHeaderTypeDef*, const uint8_t*);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef*, uint8_t*);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef*, uint32_t);
