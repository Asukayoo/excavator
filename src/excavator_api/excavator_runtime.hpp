#pragma once

#include <can/internal/excavator_canlib.hpp>
#include <can/internal/imu_canlib.hpp>
#include <excavator/internal/excavator_client.hpp>
#include <excavator_api/types.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace excavator_api {

class ExcavatorRuntime {
public:
    explicit ExcavatorRuntime(SessionConfig config);
    ~ExcavatorRuntime();

    bool start();
    bool stop();

    std::uint64_t loopTick() const;
    bool waitNextTick(std::uint64_t last_tick, std::chrono::milliseconds timeout, std::uint64_t& out_tick) const;

    excavator::ExcavatorClient& client() noexcept { return client_; }
    const excavator::ExcavatorClient& client() const noexcept { return client_; }

    const SessionConfig& config() const noexcept { return config_; }
    std::string lastError() const;

private:
    SessionConfig config_{};
    canlib::CanLib can_;
    canlib::ImuCanLib imu_;
    excavator::ExcavatorClient client_;
    bool started_{false};
    mutable std::mutex mu_{};
    std::string last_error_{};
};

class RuntimeRegistry {
public:
    static std::shared_ptr<ExcavatorRuntime> acquire(const SessionConfig& config, std::string& err);
    static void releaseUnused();

private:
    static std::string keyOf(const SessionConfig& config);
};

bool fillRefState(const excavator::ExcavatorState& in, RefState& out);
bool fillRespState(const excavator::ExcavatorState& in, RespState& out);
bool fillCommand(const SpeedScalarCmd& in, excavator::ExcavatorCommand& out);
bool fillCommand(const VelocityCmd& in, excavator::ExcavatorCommand& out);
bool fillCommand(const PositionCmd& in, excavator::ExcavatorCommand& out);
excavator::ExcavatorControlModeType toInternalMode(ControlMode mode);

}  // namespace excavator_api
