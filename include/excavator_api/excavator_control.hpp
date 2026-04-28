#pragma once

#include <excavator_api/types.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace excavator_api {

class ExcavatorControl {
public:
    ExcavatorControl();
    ~ExcavatorControl();

    bool connect(const SessionConfig& config);
    bool start();
    bool stop();
    bool close();

    bool setControlMode(ControlMode mode);
    bool sendCommand(const SpeedScalarCmd& cmd);
    bool sendCommand(const VelocityCmd& cmd);
    bool sendCommand(const PositionCmd& cmd);
    bool setPidVectors(const std::vector<std::vector<double>>& pid_vectors);
    bool applyStatusToggleMask(std::uint16_t toggle_mask);

    bool setIgnition(bool enabled);
    bool setFlameout(bool enabled);
    bool setCrush(bool enabled);
    bool setChassisLight(bool enabled);
    bool setRemoteMode(bool enabled);
    bool setPilot(bool enabled);
    bool setHighSpeed(bool enabled);
    bool setChassisDozerMode(bool enabled);
    bool setHorn(bool enabled);
    bool setMotorGear(int gear);  // 0~3
    bool setEstop(bool enabled);

    std::uint64_t loopTick() const;
    std::string lastError() const;

private:
    bool setStatusBit(int idx, bool enabled);

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace excavator_api
