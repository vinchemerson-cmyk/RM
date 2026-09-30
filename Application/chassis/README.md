# 底盘控制（规划）

当前没有底盘控制代码，也没有初始化开关。实现后再将源文件加入根目录的 `CMakeLists.txt`，并在 `Application/Robot.cpp` 接入。

计划：注册四台 M3508（反馈 ID 需按实际接线确认）；根据底盘构型做运动学分解；通过 interboard 接收云台板的速度指令；链路掉线或 `safety::ShouldOutput()` 为 false 时停止输出。实车参数集中放在 `ChassisConfig.hpp`。初始化顺序应晚于双板链路和安全模块。
