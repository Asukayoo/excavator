#pragma once
#include "data_types.hpp"

/** @file state_interface.hpp 依赖：data_types.hpp */

/**
 * Ref：参考/期望状态；Resp：反馈经转换后的响应状态。
 * 具体存储可由实现映射到共享内存。
 * 获取本接口指针由 DriverInterface::mutableState / state 提供，不在此声明。
 */
class StateInterface {
public:
    virtual ~StateInterface() = default;

    virtual bool setControlMode(const ControlMode& mode) = 0;
    virtual bool getControlMode(ControlMode& mode) const = 0;

    virtual bool setDeviceState(const DeviceState& state) = 0;
    virtual bool getDeviceState(DeviceState& state) const = 0;

    virtual bool setRef(const RobotState& s) = 0;
    virtual bool getRef(RobotState& s) const = 0;

    virtual bool setResp(const RobotState& s) = 0;
    virtual bool getResp(RobotState& s) const = 0;
};
