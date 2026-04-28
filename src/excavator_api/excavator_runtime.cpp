#include "excavator_runtime.hpp"

#include <hal/state_interface.hpp>

#include <chrono>
#include <thread>

namespace excavator_api {
namespace {

using clock_t = std::chrono::steady_clock;

std::unordered_map<std::string, std::weak_ptr<ExcavatorRuntime>>& runtime_map() {
    static std::unordered_map<std::string, std::weak_ptr<ExcavatorRuntime>> map;
    return map;
}

std::mutex& runtime_map_mu() {
    static std::mutex mu;
    return mu;
}

std::uint64_t nowNs() {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(clock_t::now().time_since_epoch());
    return static_cast<std::uint64_t>(ns.count());
}

}  // namespace

ExcavatorRuntime::ExcavatorRuntime(SessionConfig config)
    : config_(std::move(config)),
      can_(config_.can_if_name, config_.can_shm_name, config_.create_mapping),
      imu_(config_.imu_if_name, config_.imu_shm_name, config_.create_mapping),
      client_(config_.can_shm_name, false, config_.imu_shm_name) {}

ExcavatorRuntime::~ExcavatorRuntime() {
    (void)stop();
}

bool ExcavatorRuntime::start() {
    std::lock_guard<std::mutex> lock(mu_);
    if (started_) {
        return true;
    }
    last_error_.clear();

    can_.setSimulationEnabled(config_.can_simulation);
    can_.setCanBusEnabled(config_.can_bus_enabled);
    imu_.setSimulationEnabled(config_.imu_simulation);

    if (!can_.open() || !can_.start()) {
        last_error_ = "CanLib 启动失败: " + can_.lastError();
        (void)can_.stop();
        (void)can_.close();
        return false;
    }
    if (!imu_.open() || !imu_.start()) {
        last_error_ = "ImuCanLib 启动失败: " + imu_.lastError();
        (void)can_.stop();
        (void)can_.close();
        (void)imu_.stop();
        (void)imu_.close();
        return false;
    }
    if (!client_.init() || !client_.start()) {
        last_error_ = "ExcavatorClient 启动失败";
        (void)imu_.stop();
        (void)imu_.close();
        (void)can_.stop();
        (void)can_.close();
        return false;
    }
    started_ = true;
    return true;
}

bool ExcavatorRuntime::stop() {
    std::lock_guard<std::mutex> lock(mu_);
    if (!started_) {
        return true;
    }
    (void)client_.stop();
    (void)imu_.stop();
    (void)imu_.close();
    (void)can_.stop();
    (void)can_.close();
    started_ = false;
    return true;
}

std::uint64_t ExcavatorRuntime::loopTick() const {
    return client_.loopTick();
}

bool ExcavatorRuntime::waitNextTick(std::uint64_t last_tick,
                                    std::chrono::milliseconds timeout,
                                    std::uint64_t& out_tick) const {
    const auto deadline = clock_t::now() + timeout;
    while (clock_t::now() < deadline) {
        const std::uint64_t tick = client_.loopTick();
        if (tick > last_tick) {
            out_tick = tick;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    out_tick = client_.loopTick();
    return false;
}

std::string ExcavatorRuntime::lastError() const {
    std::lock_guard<std::mutex> lock(mu_);
    if (!last_error_.empty()) {
        return last_error_;
    }
    return can_.lastError();
}

std::shared_ptr<ExcavatorRuntime> RuntimeRegistry::acquire(const SessionConfig& config, std::string& err) {
    std::lock_guard<std::mutex> lock(runtime_map_mu());
    const std::string key = keyOf(config);
    auto& map = runtime_map();
    auto it = map.find(key);
    if (it != map.end()) {
        if (auto p = it->second.lock()) {
            err.clear();
            return p;
        }
        map.erase(it);
    }
    auto p = std::make_shared<ExcavatorRuntime>(config);
    map.emplace(key, p);
    err.clear();
    return p;
}

void RuntimeRegistry::releaseUnused() {
    std::lock_guard<std::mutex> lock(runtime_map_mu());
    auto& map = runtime_map();
    for (auto it = map.begin(); it != map.end();) {
        if (it->second.expired()) {
            it = map.erase(it);
        } else {
            ++it;
        }
    }
}

std::string RuntimeRegistry::keyOf(const SessionConfig& config) {
    return config.can_if_name + "|" + config.imu_if_name + "|" + config.can_shm_name + "|" + config.imu_shm_name +
           "|" + (config.create_mapping ? "1" : "0") + "|" + (config.can_simulation ? "1" : "0") + "|" +
           (config.imu_simulation ? "1" : "0") + "|" + (config.can_bus_enabled ? "1" : "0");
}

bool fillRefState(const excavator::ExcavatorState& in, RefState& out) {
    out.position = in.position;
    out.velocity = in.velocity;
    out.acceleration = in.acceleration;
    out.velocity_scalar = in.velocity_scalar;
    out.status = in.status;
    out.motor_rpm = in.motor_rpm;
    out.plan_rpm = in.plan_rpm;
    return true;
}

bool fillRespState(const excavator::ExcavatorState& in, RespState& out) {
    out.position = in.position;
    out.velocity = in.velocity;
    out.acceleration = in.acceleration;
    out.velocity_scalar = in.velocity_scalar;
    out.status = in.status;
    out.motor_rpm = in.motor_rpm;
    out.plan_rpm = in.plan_rpm;
    return true;
}

bool fillCommand(const SpeedScalarCmd& in, excavator::ExcavatorCommand& out) {
    out.speed_scalar = in.speed_scalar;
    return true;
}

bool fillCommand(const VelocityCmd& in, excavator::ExcavatorCommand& out) {
    out.velocity = in.velocity;
    return true;
}

bool fillCommand(const PositionCmd& in, excavator::ExcavatorCommand& out) {
    out.position = in.position;
    return true;
}

excavator::ExcavatorControlModeType toInternalMode(ControlMode mode) {
    switch (mode) {
        case ControlMode::OpenLoopMotorSpeed:
            return excavator::ExcavatorControlModeType::OpenLoopMotorSpeed;
        case ControlMode::ClosedLoopJointPosition:
            return excavator::ExcavatorControlModeType::ClosedLoopJointPosition;
        case ControlMode::ClosedLoopJointVelocity:
            return excavator::ExcavatorControlModeType::ClosedLoopJointVelocity;
        case ControlMode::ClosedLoopVelocityScalar:
            return excavator::ExcavatorControlModeType::ClosedLoopVelocityScalar;
        default:
            return excavator::ExcavatorControlModeType::OpenLoopMotorSpeed;
    }
}

}  // namespace excavator_api
