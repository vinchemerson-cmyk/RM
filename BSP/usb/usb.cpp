#include "usb.hpp"
#include "memory_layout.h"

DMA_BUFFER
std::array<usb, usbConfig::MAX_INSTANCE> usb::_usb_instance_table{};
