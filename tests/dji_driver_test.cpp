/** @file dji_driver_test.cpp @brief 执行实际 DJI/CAN 驱动，HAL 替身记录帧及初始化状态。 */
#include <cassert>
#include <array>
#include <cstring>
#include <deque>
#include <vector>
#include <iostream>
#include "motor/dji/dji.h"
#include "motor/dji/dji_protocol.h"
#include "gimbal/GimbalConfig.hpp"
#include "shoot/ShootConfig.hpp"

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
static uint32_t tick = 100U;
static uint32_t globalFilterCalls = 0U;
static bool fifoFull = false;
static bool readFailure = false;
struct SentFrame { FDCAN_HandleTypeDef* bus; uint32_t id; std::array<uint8_t, 8> bytes; };
static std::vector<SentFrame> sent;
struct ReceivedFrame { uint32_t filter; std::array<uint8_t, 8> bytes; };
static std::deque<ReceivedFrame> received;
uint32_t osKernelGetTickCount() { return tick; }
void Error_Handler() { assert(false && "Unexpected initialization failure"); }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef* h, const FDCAN_FilterTypeDef* f)
{ assert(f->FilterIndex < h->Init.StdFiltersNbr); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef* h, uint32_t, uint32_t, uint32_t, uint32_t)
{ ++globalFilterCalls; return h->State == HAL_FDCAN_STATE_READY ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t) { return HAL_OK; }
HAL_FDCAN_StateTypeDef HAL_FDCAN_GetState(FDCAN_HandleTypeDef* h) { return h->State; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef* h)
{ assert(h->State == HAL_FDCAN_STATE_READY); h->State = HAL_FDCAN_STATE_BUSY; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef* h, const FDCAN_TxHeaderTypeDef* header, const uint8_t* data)
{
    if (fifoFull) return HAL_ERROR;
    assert(header->DataLength == 8U && header->FDFormat == FDCAN_CLASSIC_CAN);
    SentFrame frame{h, header->Identifier, {}};
    std::memcpy(frame.bytes.data(), data, 8U); sent.push_back(frame); return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef*, uint32_t) { return received.size(); }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef* h, uint8_t* bytes)
{
    if (received.empty() || readFailure) return HAL_ERROR;
    h->FilterIndex = received.front().filter; h->DataLength = 8U;
    std::memcpy(bytes, received.front().bytes.data(), 8U); received.pop_front(); return HAL_OK;
}
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t);

static DJI_Motor_s* Make(Motor_Type_e type, uint32_t id, FDCAN_HandleTypeDef* bus, float current)
{
    Motor_Init_Config_s c{}; c.motor_type = type;
    c.can_init_config.can_handle = bus; c.can_init_config.rx_id = id;
    c.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    auto* motor = DJI_Motor_Init(c);
    if (motor != nullptr) {
        DJI_Motor_Enable(motor); motor->motor_settings.current = current;
        motor->feedback.angle_initialized = true; motor->feedback.last_update_tick = tick;
        motor->can_instance->output.rx_len = 8U;
    }
    return motor;
}
static int16_t Slot(const SentFrame& frame, uint8_t slot)
{ return static_cast<int16_t>((frame.bytes[slot * 2U] << 8U) | frame.bytes[slot * 2U + 1U]); }

int main()
{
    DjiCommandAddress address{};
    for (uint32_t id = 1U; id <= 7U; ++id) {
        assert(DjiResolveCommand(true, 0x204U + id, address));
        assert(address.frame_id == (id <= 4U ? 0x1FEU : 0x2FEU));
        assert(address.slot == (id - 1U) % 4U);
    }
    assert(DjiResolveCommand(false, 0x208U, address) && address.frame_id == 0x1FFU && address.slot == 3U);
    assert(!DjiResolveCommand(true, 0x20CU, address));
    assert(!DjiResolveCommand(false, 0x200U, address));
    auto* yaw = Make(GM6020, 0x205U, &hfdcan1, 1111.0F);
    auto* pitch = Make(GM6020, 0x206U, &hfdcan1, -2222.0F);
    auto* feeder = Make(M2006, 0x201U, &hfdcan1, 20000.0F);
    auto* right = Make(M3508, 0x202U, &hfdcan1, 3333.0F);
    auto* left = Make(M3508, 0x203U, &hfdcan1, -4444.0F);
    auto* otherBus = Make(M3508, 0x202U, &hfdcan3, 5555.0F);
    auto* gm5 = Make(GM6020, 0x209U, &hfdcan1, 6666.0F);
    assert(yaw && pitch && feeder && right && left && otherBus && gm5);
    assert(globalFilterCalls == 2U); // 每条总线一次，不因注册第二台电机而失败
    assert(!Make(M3508, 0x202U, &hfdcan1, 0.0F)); // 同总线重复反馈 ID
    assert(!Make(GM6020, 0x20CU, &hfdcan1, 0.0F));
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    assert(!Make(M3508, 0x201U, &hfdcan2, 0.0F));
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    DJI_Motor_s* motors[] = {yaw, pitch, feeder, right, left, otherBus, gm5};
    DJI_Motor_Control(motors, 7U, 0.001F);
    assert(sent.size() == 4U);
    assert(sent[0].id == 0x1FEU && Slot(sent[0], 0U) == 1111 && Slot(sent[0], 1U) == -2222);
    assert(sent[1].id == 0x200U && Slot(sent[1], 0U) == 10000); // C610 限幅
    assert(Slot(sent[1], 1U) == 3333 && Slot(sent[1], 2U) == -4444); // 混合型号同帧
    assert(sent[2].bus == &hfdcan3 && Slot(sent[2], 1U) == 5555); // 两总线互不覆盖
    assert(sent[3].id == 0x2FEU && Slot(sent[3], 0U) == 6666);
    for (const auto& frame : sent) assert(frame.id != 0x1FFU && frame.id != 0x2FFU);
    yaw->motor_settings.current = 30000.0F; pitch->motor_settings.current = -30000.0F;
    gm5->motor_settings.current = -30000.0F;
    sent.clear(); DJI_Motor_Control(motors, 7U, 0.001F);
    assert(Slot(sent[0], 0U) == 16384 && Slot(sent[0], 1U) == -16384);
    assert(Slot(sent[3], 0U) == -16384 && Slot(sent[3], 1U) == 0);
    yaw->motor_settings.current = 1111.0F; pitch->motor_settings.current = -2222.0F;
    gm5->motor_settings.current = 6666.0F;
    DJI_Motor_Disable(feeder); right->feedback.last_update_tick = tick - 101U;
    sent.clear(); DJI_Motor_Control(motors, 7U, 0.001F);
    assert(Slot(sent[1], 0U) == 0 && Slot(sent[1], 1U) == 3333 && Slot(sent[1], 2U) == -4444);
    sent.clear(); fifoFull = true; DJI_Motor_Control(motors, 7U, 0.001F);
    assert(sent.empty()); fifoFull = false;
    DJI_Motor_Control(motors, 7U, 0.001F); assert(sent.size() == 4U);
    received.push_back({0U, {0x01, 0x02, 0x00, 0x01, 0x20, 0x00, 30, 0}});
    received.push_back({1U, {0x02, 0x03, 0x00, 0x02, 0xE0, 0x00, 31, 0}});
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(received.empty() && yaw->feedback.raw_angle == 0x102 && pitch->feedback.raw_angle == 0x203);
    assert(yaw->feedback.raw_current == 8192 && yaw->feedback.current == 1.5F);
    assert(pitch->feedback.raw_current == -8192 && pitch->feedback.current == -1.5F);
    received.push_back({0U, {}}); readFailure = true;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(received.size() == 1U); received.clear(); readFailure = false;
    auto* reversed = Make(M3508, 0x203U, &hfdcan3, 0.0F);
    assert(reversed);
    reversed->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    reversed->motor_settings.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE;
    reversed->can_instance->output.rx_buffer = {0, 1, 0xFF, 0x9C, 0x40, 0x00, 30, 0}; // raw speed=-100 RPM
    reversed->can_instance->attribute.can_callback_func(&reversed->can_instance->attribute);
    assert(reversed->feedback.speed == 600.0F);
    assert(reversed->feedback.raw_current == 16384 && reversed->feedback.current == -10.0F);
    assert(reversed->controller.speed_PID.Configure({1.0F, 0.0F, 0.0F, -8000.0F, 8000.0F, 0.0F, 0.0F}));
    reversed->motor_settings.close_loop_type = SPEED_AND_CURRENT_LOOP;
    reversed->motor_settings.speed = 1200.0F;
    sent.clear(); DJI_Motor_s* reverseList[]{reversed};
    DJI_Motor_Control(reverseList, 1U, 0.001F);
    assert(sent.size() == 1U && Slot(sent[0], 2U) == -600); // 反装电机也保持负反馈
    assert(reversed->controller.angle_PID.Configure({1.0F, 0.0F, 0.0F, -2000.0F, 2000.0F, 0.0F, 0.0F}));
    reversed->motor_settings.close_loop_type = ANGLE_AND_SPEED_LOOP;
    reversed->motor_settings.angle = reversed->feedback.total_angle + 1000.0F;
    reversed->motor_settings.angle_epoch = reversed->feedback.angle_epoch - 1U;
    sent.clear(); DJI_Motor_Control(reverseList, 1U, 0.001F);
    assert(Slot(sent[0], 2U) == 0); // 应用快照后中断重锚，旧位置目标不得再发送
    reversed->motor_settings.angle_epoch = reversed->feedback.angle_epoch;
    sent.clear(); DJI_Motor_Control(reverseList, 1U, 0.001F);
    assert(Slot(sent[0], 2U) == -400);
    const uint32_t previousEpoch = reversed->feedback.angle_epoch;
    const float previousAngle = reversed->feedback.total_angle;
    tick += 1000U;
    reversed->can_instance->output.rx_buffer[1] = 2U;
    reversed->can_instance->attribute.can_callback_func(&reversed->can_instance->attribute);
    assert(reversed->enabled && reversed->feedback.angle_epoch == previousEpoch);
    assert(std::fabs(reversed->feedback.total_angle - previousAngle + 360.0F / 8192.0F) < 0.001F);

    // C620/C610 ID5..8仍用0x1FF，不因GM6020改电流协议而变化。
    auto* c620Id5 = Make(M3508, 0x205U, &hfdcan2, 7000.0F);
    auto* c610Id6 = Make(M2006, 0x206U, &hfdcan2, -9000.0F);
    assert(c620Id5 && c610Id6);
    DJI_Motor_s* upperIds[]{c620Id5, c610Id6, gm5};
    sent.clear(); DJI_Motor_Control(upperIds, 3U, 0.001F);
    assert(sent.size() == 2U && sent[0].bus == &hfdcan2 && sent[0].id == 0x1FFU);
    assert(Slot(sent[0], 0U) == 7000 && Slot(sent[0], 1U) == -9000);
    assert(sent[1].bus == &hfdcan1 && sent[1].id == 0x2FEU);

    // pitch/yaw均启用位置模式；拨弹仍关闭。下方单独测试pitch速度环的电流输出。
    assert(kEnableYaw && kEnablePitch && !shoot::kEnableFeeder);
    assert(kPitchPositionMode && kYawPositionMode);
    assert(!kEncoderCalibrationOnly && kReadYawEncoder);
    assert(kPitchAngleLimits.IsConfigured() && kYawAngleLimits.IsConfigured());
    assert(kGimbalCanHandle == &hfdcan1 && shoot::kCanHandle == &hfdcan2);
    assert(kPitchMotorFeedbackId == 0x205U && kYawMotorFeedbackId == 0x206U);
    assert(shoot::kLeftFeedbackId == 0x201U && shoot::kRightFeedbackId == 0x202U);
    auto* benchRight = Make(M3508, shoot::kRightFeedbackId, shoot::kCanHandle, 0.0F);
    auto* benchLeft = Make(M3508, shoot::kLeftFeedbackId, shoot::kCanHandle, 0.0F);
    assert(benchRight && benchLeft);
    for (auto* wheel : {benchRight, benchLeft}) {
        wheel->feedback = {}; wheel->can_instance->output.rx_len = 0U;
        assert(wheel->controller.speed_PID.Configure(shoot::kWheelSpeedPid));
        wheel->motor_settings.close_loop_type = SPEED_AND_CURRENT_LOOP;
        wheel->motor_settings.speed = 1200.0F;
    }
    benchLeft->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    auto* benchPitch = yaw; // 前面创建的 GM6020 ID1，现用作实物 pitch 的测试句柄。
    benchPitch->feedback = {}; benchPitch->can_instance->output.rx_len = 0U;
    benchPitch->motor_settings.current = 1234.0F;
    DJI_Motor_s* benchMotors[]{benchPitch, benchRight, benchLeft};
    sent.clear(); DJI_Motor_Control(benchMotors, 3U, 0.001F);
    assert(sent.size() == 2U); // 未收到首帧也发送指令
    assert(sent[0].bus == &hfdcan1 && sent[0].id == 0x1FEU);
    assert(Slot(sent[0], 0U) == 1234 && Slot(sent[0], 1U) == 0);
    assert(sent[1].bus == &hfdcan2 && sent[1].id == 0x200U);
    // 使用当前实车配置计算期望值，不把旧摩擦轮Kp=0.4对应的480写死。
    PIDInstance wheelReference{};
    assert(wheelReference.Configure(shoot::kWheelSpeedPid));
    const int16_t expectedWheel = static_cast<int16_t>(wheelReference.Update(1200.0F, 0.0F, 0.001F));
    assert(expectedWheel > 0 && Slot(sent[1], 0U) == -expectedWheel &&
           Slot(sent[1], 1U) == expectedWheel && Slot(sent[1], 2U) == 0);
    tick += 10000U;
    sent.clear(); DJI_Motor_Control(benchMotors, 3U, 0.001F);
    assert(Slot(sent[0], 0U) == 1234 && Slot(sent[1], 0U) != 0 && Slot(sent[1], 1U) != 0);

    // 实际云台速度环：按用户当前增益输出电流码，应用上限约0.5A。
    assert(benchPitch->controller.speed_PID.Configure({kPitchSpeedKp, kPitchSpeedKi, kPitchSpeedKd,
        -kGimbalOutputLimit, kGimbalOutputLimit, -kGimbalIntegralLimit, kGimbalIntegralLimit}));
    benchPitch->motor_settings.close_loop_type = SPEED_AND_CURRENT_LOOP;
    benchPitch->motor_settings.current = 0.0F;
    benchPitch->motor_settings.speed = kPitchMaxSpeedDps;
    sent.clear(); DJI_Motor_Control(benchMotors, 3U, 0.001F);
    assert(sent[0].id == 0x1FEU && Slot(sent[0], 0U) > 0 &&
           Slot(sent[0], 0U) <= static_cast<int16_t>(kGimbalOutputLimit));
    assert(Slot(sent[0], 1U) == 0); // yaw保持零电流
    benchPitch->motor_settings.speed = 10000.0F;
    sent.clear(); DJI_Motor_Control(benchMotors, 3U, 0.001F);
    assert(Slot(sent[0], 0U) == static_cast<int16_t>(kGimbalOutputLimit));
    benchPitch->motor_settings.speed = -10000.0F;
    sent.clear(); DJI_Motor_Control(benchMotors, 3U, 0.001F);
    assert(Slot(sent[0], 0U) == -static_cast<int16_t>(kGimbalOutputLimit));
    // 使用真实应用参数验证pitch速度模式、yaw位置模式共用0x1FE，不互相覆盖。
    auto* benchYaw = pitch; // 前面创建的GM6020 ID2。
    assert(benchPitch->controller.angle_PID.Configure(kPitchAnglePid));
    assert(benchYaw->controller.angle_PID.Configure(kYawAnglePid));
    assert(benchYaw->controller.speed_PID.Configure({kYawSpeedKp, kYawSpeedKi, kYawSpeedKd,
        -kGimbalOutputLimit, kGimbalOutputLimit, -kGimbalIntegralLimit, kGimbalIntegralLimit}));
    DJI_Motor_s* benchAll[]{benchPitch, benchYaw, benchRight, benchLeft};
    for (auto* axis : {benchPitch, benchYaw}) {
        DJI_Motor_Disable(axis);
        axis->feedback.angle_initialized = true;
        axis->feedback.total_angle = 100.0F;
        axis->feedback.speed = 0.0F;
        axis->motor_settings.close_loop_type = axis == benchPitch ? SPEED_AND_CURRENT_LOOP : ANGLE_AND_SPEED_LOOP;
        axis->motor_settings.speed = 5.0F;
        axis->motor_settings.angle = 120.5F; // 当前yaw速度Kp较小，确保量化后仍有非零指令。
        axis->motor_settings.angle_epoch = axis->feedback.angle_epoch;
        DJI_Motor_Enable(axis);
    }
    sent.clear(); DJI_Motor_Control(benchAll, 4U, 0.001F);
    assert(sent.size() == 2U && sent[0].id == 0x1FEU && sent[0].bus == &hfdcan1);
    const int16_t stationaryPitch = Slot(sent[0], 0U);
    assert(stationaryPitch > 0 && Slot(sent[0], 1U) > 0);
    assert(stationaryPitch < static_cast<int16_t>(kGimbalOutputLimit));
    assert(Slot(sent[0], 1U) < static_cast<int16_t>(kGimbalOutputLimit));
    // 一格RPM反馈变化不会把低速目标的电流推到正/负满限幅。
    benchPitch->feedback.speed = 6.0F;
    sent.clear(); DJI_Motor_Control(benchAll, 4U, 0.001F);
    assert(std::abs(Slot(sent[0], 0U)) < static_cast<int16_t>(kGimbalOutputLimit));
    assert(std::abs(Slot(sent[0], 0U) - stationaryPitch) <
           static_cast<int16_t>(kGimbalOutputLimit));
    // 纯速度环不追逐旧角度：松杆且测得静止时输出零；转速匹配时比例输出也为零。
    benchPitch->feedback.speed = 0.0F;
    benchPitch->motor_settings.speed = 0.0F;
    benchPitch->motor_settings.angle = -10000.0F;
    sent.clear(); DJI_Motor_Control(benchAll, 4U, 0.001F);
    assert(Slot(sent[0], 0U) == 0 && Slot(sent[0], 1U) > 0);
    benchPitch->motor_settings.speed = kPitchMaxSpeedDps;
    benchPitch->feedback.speed = kPitchMaxSpeedDps;
    sent.clear(); DJI_Motor_Control(benchAll, 4U, 0.001F);
    assert(Slot(sent[0], 0U) == 0 && Slot(sent[0], 1U) > 0);
    for (auto* motor : benchAll) DJI_Motor_Disable(motor);
    sent.clear(); DJI_Motor_Control(benchAll, 4U, 0.001F);
    assert(Slot(sent[0], 0U) == 0 && Slot(sent[0], 1U) == 0 &&
           Slot(sent[1], 0U) == 0 && Slot(sent[1], 1U) == 0);
    assert(sent[0].id == 0x1FEU); // 停机帧仍为电流协议
    std::cout << "Actual DJI/CAN driver: GM current frames/limits, bench CAN1/CAN2, no-feedback output, FIFO recovery: PASS\n";
}
