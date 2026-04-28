#pragma once
#include "data_types.hpp"
#include "state_interface.hpp"

/** @file driver_interface.hpp 依赖：state_interface.hpp → data_types.hpp */

class DriverInterface {
public:
    virtual ~DriverInterface() = default;

    virtual bool init() = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual bool reset() = 0;

    virtual StateInterface* mutableState() = 0;
    virtual const StateInterface* state() const = 0;

    virtual const DeviceState& getDeviceState() const = 0;

protected:
    virtual void loop() = 0;
    virtual bool update() = 0;
};
