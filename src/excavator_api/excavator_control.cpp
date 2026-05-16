#include <excavator_api/excavator_control.hpp>

#include "excavator_runtime.hpp"

#include <excavator/internal/excavator_data_type.hpp>

namespace excavator_api {

struct ExcavatorControl::Impl {
    std::shared_ptr<ExcavatorRuntime> runtime{};
    std::string last_error{};
};

ExcavatorControl::ExcavatorControl() : impl_(std::make_unique<Impl>()) {}

ExcavatorControl::~ExcavatorControl() = default;

bool ExcavatorControl::connect(const SessionConfig& config) {
    std::string err;
    auto runtime = RuntimeRegistry::acquire(config, err);
    if (!runtime) {
        impl_->last_error = err;
        return false;
    }
    impl_->runtime = std::move(runtime);
    impl_->last_error.clear();
    return true;
}

bool ExcavatorControl::start() {
    if (!impl_->runtime) {
        impl_->last_error = "runtime 未连接";
        return false;
    }
    if (!impl_->runtime->start()) {
        impl_->last_error = impl_->runtime->lastError();
        return false;
    }
    return true;
}

bool ExcavatorControl::stop() {
    if (!impl_->runtime) {
        return true;
    }
    return impl_->runtime->stop();
}

bool ExcavatorControl::close() {
    (void)stop();
    impl_->runtime.reset();
    RuntimeRegistry::releaseUnused();
    return true;
}

bool ExcavatorControl::setControlMode(ControlMode mode) {
    if (!impl_->runtime) return false;
    excavator::ExcavatorControlMode m{};
    m.mode = toInternalMode(mode);
    return impl_->runtime->client().mutableState()->setControlMode(m);
}

bool ExcavatorControl::sendCommand(const SpeedScalarCmd& cmd) {
    if (!impl_->runtime) {
        impl_->last_error = "runtime 未连接";
        return false;
    }
    excavator::ExcavatorControlMode mode{};
    if (!impl_->runtime->client().state()->getControlMode(mode)) {
        impl_->last_error = "读取控制模式失败";
        return false;
    }
    if (mode.mode != excavator::ExcavatorControlModeType::OpenLoopMotorSpeed &&
        mode.mode != excavator::ExcavatorControlModeType::ClosedLoopVelocityScalar) {
        impl_->last_error = "SpeedScalarCmd 仅允许 OpenLoopMotorSpeed/ClosedLoopVelocityScalar 模式";
        return false;
    }
    excavator::ExcavatorCommand out{};
    (void)fillCommand(cmd, out);
    impl_->runtime->client().submitServo(out);
    impl_->last_error.clear();
    return true;
}

bool ExcavatorControl::sendCommand(const VelocityCmd& cmd) {
    if (!impl_->runtime) {
        impl_->last_error = "runtime 未连接";
        return false;
    }
    excavator::ExcavatorControlMode mode{};
    if (!impl_->runtime->client().state()->getControlMode(mode)) {
        impl_->last_error = "读取控制模式失败";
        return false;
    }
    if (mode.mode != excavator::ExcavatorControlModeType::ClosedLoopJointVelocity) {
        impl_->last_error = "VelocityCmd 仅允许 ClosedLoopJointVelocity 模式";
        return false;
    }
    excavator::ExcavatorCommand out{};
    (void)fillCommand(cmd, out);
    impl_->runtime->client().submitServo(out);
    impl_->last_error.clear();
    return true;
}

bool ExcavatorControl::sendCommand(const PositionCmd& cmd) {
    if (!impl_->runtime) {
        impl_->last_error = "runtime 未连接";
        return false;
    }
    excavator::ExcavatorControlMode mode{};
    if (!impl_->runtime->client().state()->getControlMode(mode)) {
        impl_->last_error = "读取控制模式失败";
        return false;
    }
    if (mode.mode != excavator::ExcavatorControlModeType::ClosedLoopJointPosition) {
        impl_->last_error = "PositionCmd 仅允许 ClosedLoopJointPosition 模式";
        return false;
    }
    excavator::ExcavatorCommand out{};
    (void)fillCommand(cmd, out);
    impl_->runtime->client().submitServo(out);
    impl_->last_error.clear();
    return true;
}

bool ExcavatorControl::setPidVectors(const std::vector<std::vector<double>>& pid_vectors) {
    if (!impl_->runtime) {
        impl_->last_error = "runtime 未连接";
        return false;
    }
    if (!impl_->runtime->client().setPidVectors(pid_vectors)) {
        impl_->last_error = "PID 参数格式非法，要求 9x8 有限数值向量";
        return false;
    }
    impl_->last_error.clear();
    return true;
}

bool ExcavatorControl::applyStatusToggleMask(std::uint16_t toggle_mask) {
    if (!impl_->runtime) return false;
    impl_->runtime->client().applyStatusToggleMask(toggle_mask);
    return true;
}

bool ExcavatorControl::setStatusBit(int idx, bool enabled) {
    if (!impl_->runtime) return false;
    excavator::ExcavatorState ref{};
    if (!impl_->runtime->client().state()->getRef(ref)) {
        return false;
    }
    int cur = ref.status(idx);
    if (idx == 9) {
        cur &= 0x3;
        int desired = enabled ? 1 : 0;
        if (cur == desired) return true;
        impl_->runtime->client().toggleMotorGear();
        return true;
    }
    const int target = enabled ? 1 : 0;
    if (cur == target) return true;
    switch (idx) {
        case 0: impl_->runtime->client().toggleIgnition(); return true;
        case 1: impl_->runtime->client().toggleFlameout(); return true;
        case 2: impl_->runtime->client().toggleCrush(); return true;
        case 3: impl_->runtime->client().toggleChassisLight(); return true;
        case 4: impl_->runtime->client().toggleRemoteMode(); return true;
        case 5: impl_->runtime->client().togglePilot(); return true;
        case 6: impl_->runtime->client().toggleHighSpeed(); return true;
        case 7: impl_->runtime->client().toggleChassisDozerMode(); return true;
        case 8: impl_->runtime->client().toggleHorn(); return true;
        case 10: impl_->runtime->client().toggleEstop(); return true;
        default: return false;
    }
}

bool ExcavatorControl::setIgnition(bool enabled) { return setStatusBit(0, enabled); }
bool ExcavatorControl::setFlameout(bool enabled) { return setStatusBit(1, enabled); }
bool ExcavatorControl::setCrush(bool enabled) { return setStatusBit(2, enabled); }
bool ExcavatorControl::setChassisLight(bool enabled) { return setStatusBit(3, enabled); }
bool ExcavatorControl::setRemoteMode(bool enabled) { return setStatusBit(4, enabled); }
bool ExcavatorControl::setPilot(bool enabled) { return setStatusBit(5, enabled); }
bool ExcavatorControl::setHighSpeed(bool enabled) { return setStatusBit(6, enabled); }
bool ExcavatorControl::setChassisDozerMode(bool enabled) { return setStatusBit(7, enabled); }
bool ExcavatorControl::setHorn(bool enabled) { return setStatusBit(8, enabled); }
bool ExcavatorControl::setEstop(bool enabled) { return setStatusBit(10, enabled); }

bool ExcavatorControl::setMotorGear(int gear) {
    if (!impl_->runtime) return false;
    if (gear < 0 || gear > 3) return false;
    excavator::ExcavatorState ref{};
    if (!impl_->runtime->client().state()->getRef(ref)) return false;
    int cur = ref.status(9) & 0x3;
    while (cur != gear) {
        impl_->runtime->client().toggleMotorGear();
        cur = (cur + 1) & 0x3;
    }
    return true;
}

std::uint64_t ExcavatorControl::loopTick() const {
    if (!impl_->runtime) return 0;
    return impl_->runtime->loopTick();
}

std::string ExcavatorControl::lastError() const {
    if (!impl_->last_error.empty()) return impl_->last_error;
    if (!impl_->runtime) return "runtime 未连接";
    return impl_->runtime->lastError();
}

}  // namespace excavator_api
