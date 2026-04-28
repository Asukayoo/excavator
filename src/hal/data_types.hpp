#pragma once
#include "abstract_type.hpp"

/** @file data_types.hpp 依赖：abstract_type.hpp */

class RobotCommand : public AbstractType {};
class HardwareCommand : public AbstractType {};
class HardwareState : public AbstractType {};
class RobotState : public AbstractType {};
class ControlMode : public AbstractType {};
class DeviceState : public AbstractType {};
