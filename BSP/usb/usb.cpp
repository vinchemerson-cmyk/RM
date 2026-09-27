/**
 * @file usb.cpp
 * @brief 保存 USB CDC 设备实例表。
 */

#include "usb.hpp"

__attribute__((section(".axi_bss")))
std::array<usb, usbConfig::MAX_INSTANCE> usb::_usb_instance_table{};
