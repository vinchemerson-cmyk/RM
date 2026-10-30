/** @file board_can_config.h @brief C/C++共用的上下板CAN配置；修改映射不自动启用执行器。 */
#pragma once
#include <stdint.h>

#ifndef INFANTRY_BOARD_GIMBAL
#define INFANTRY_BOARD_GIMBAL 1
#endif
#if INFANTRY_BOARD_GIMBAL != 0 && INFANTRY_BOARD_GIMBAL != 1
#error "INFANTRY_BOARD_GIMBAL must be 0 or 1"
#endif

enum
{
	BOARD_CAN_CLASSIC = 0,
	BOARD_CAN_FD = 1,
	BOARD_CAN_FD_BRS = 2
};
typedef struct
{
    uint32_t mode;//工作模式
	uint32_t standard_filters;//标准ID过滤器数量

    uint32_t nominal_prescaler;//
	uint32_t nominal_sjw;
	uint32_t nominal_seg1;
	uint32_t nominal_seg2;
    uint32_t data_prescaler;
	uint32_t data_sjw;
	 uint32_t data_seg1;
	uint32_t data_seg2;
} BoardCanConfig;

/* 两张表独立填写。暂保留原模式/时序：CAN1/2经典，CAN3 FD+BRS；不是已确认的实物接线表。
 * 时序基于实际FDCAN时钟计算：bitrate=clock/(prescaler*(1+seg1+seg2))，不得只看分频猜波特率。
 * DM与板间共线时，将该板对应总线设为FD/FD_BRS，双方电机/控制板均须配置相同总线时序。
 */
#if INFANTRY_BOARD_GIMBAL
static const BoardCanConfig kBoardCanConfig[3] = {
    {BOARD_CAN_FD, 8, 1,20,59,20, 1,2,13,2},
    {BOARD_CAN_CLASSIC, 4, 1,20,59,20, 1,2,13,2},
    {BOARD_CAN_FD,  4, 1,20,59,20, 1,2,13,2},
};
#ifndef INFANTRY_INTERBOARD_FDCAN
#define INFANTRY_INTERBOARD_FDCAN 1
#endif
#else
static const BoardCanConfig kBoardCanConfig[3] = {
    {BOARD_CAN_CLASSIC, 8, 1,20,59,20, 1,2,13,2},
    {BOARD_CAN_FD, 4, 1,20,59,20, 1,2,13,2},
    {BOARD_CAN_CLASSIC,  4, 1,20,59,20, 1,2,13,2},
};
#ifndef INFANTRY_INTERBOARD_FDCAN
#define INFANTRY_INTERBOARD_FDCAN 2
#endif
#endif

/* HAL的元素大小单位为32位字，已包含2字帧头；8字节元素=4字，64字节元素=18字。
 * 每路保留原队列数量：两个RX FIFO各8、专用RX Buffer为2/3/2、TX事件8、专用TX8、TX FIFO8。
 * 三路按实际用量连续分配，避免扩大FD容量后仍沿用旧MessageRAMOffset。 */
static inline uint32_t BoardCanElementWords(unsigned index)
{ return kBoardCanConfig[index].mode == BOARD_CAN_CLASSIC ? 4U : 18U; }
static inline uint32_t BoardCanRamWords(unsigned index)
{
    const uint32_t rx_buffers = index == 1U ? 3U : 2U;
    return kBoardCanConfig[index].standard_filters + 4U * 2U +
           (8U + 8U + rx_buffers + 8U + 8U) * BoardCanElementWords(index) + 8U * 2U;
}
static inline uint32_t BoardCanRamOffset(unsigned index)
{
    uint32_t result = 0U;
    for (unsigned i = 0U; i < index; ++i) result += BoardCanRamWords(i);
    return result;
}
static inline uint32_t BoardCanTotalRamWords()
{ return BoardCanRamOffset(3U); }
