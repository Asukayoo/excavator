#pragma once
#include "data_types.hpp"

/** @file converter_interface.hpp 依赖：data_types.hpp */

/** RobotCommand→RobotState→HardwareCommand；HardwareState→RobotState（入参显式，便于测试与审计） */
class ConverterInterface {
public:
    virtual ~ConverterInterface() = default;

    virtual bool robotCmdToRobotState(const RobotCommand& cmd, RobotState& state_out) = 0;
    virtual bool robotStateToHardwareCmd(const RobotState& state, HardwareCommand& cmd_out) = 0;
    virtual bool hardwareStateToRobotState(const HardwareState& raw_in, RobotState& state_out) = 0;
};
