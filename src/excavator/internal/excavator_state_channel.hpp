#pragma once

#include <excavator/internal/excavator_data_type.hpp>
#include <hal/state_interface.hpp>
#include <mutex>

namespace excavator {

class ExcavatorStateChannel final : public StateInterface {
public:
    class UnsafeAccess final {
    public:
        ExcavatorControlMode& mode() noexcept { return channel_.mode_; }
        ExcavatorDeviceState& deviceState() noexcept { return channel_.device_state_; }
        ExcavatorState& ref() noexcept { return channel_.ref_; }
        ExcavatorState& resp() noexcept { return channel_.resp_; }

    private:
        friend class ExcavatorStateChannel;
        explicit UnsafeAccess(ExcavatorStateChannel& channel) : lock_(channel.mu_), channel_(channel) {}
        std::unique_lock<std::mutex> lock_;
        ExcavatorStateChannel& channel_;
    };

    // 仅供内部实时线程使用：持锁返回可写引用，避免重复 get/set 拷贝。
    UnsafeAccess unsafeAccessForWorker() { return UnsafeAccess(*this); }

    bool setControlMode(const ControlMode& mode) override {
        std::lock_guard<std::mutex> lock(mu_);
        const auto* m = dynamic_cast<const ExcavatorControlMode*>(&mode);
        if (!m) {
            return false;
        }
        mode_ = *m;
        return true;
    }

    bool getControlMode(ControlMode& mode) const override {
        std::lock_guard<std::mutex> lock(mu_);
        auto* m = dynamic_cast<ExcavatorControlMode*>(&mode);
        if (!m) {
            return false;
        }
        *m = mode_;
        return true;
    }

    bool setDeviceState(const DeviceState& state) override {
        std::lock_guard<std::mutex> lock(mu_);
        const auto* st = dynamic_cast<const ExcavatorDeviceState*>(&state);
        if (!st) {
            return false;
        }
        device_state_ = *st;
        return true;
    }

    bool getDeviceState(DeviceState& state) const override {
        std::lock_guard<std::mutex> lock(mu_);
        auto* st = dynamic_cast<ExcavatorDeviceState*>(&state);
        if (!st) {
            return false;
        }
        *st = device_state_;
        return true;
    }

    bool setResp(const RobotState& s) override {
        std::lock_guard<std::mutex> lock(mu_);
        const auto* st = dynamic_cast<const ExcavatorState*>(&s);
        if (!st) {
            return false;
        }
        resp_ = *st;
        return true;
    }

    bool getResp(RobotState& s) const override {
        std::lock_guard<std::mutex> lock(mu_);
        auto* st = dynamic_cast<ExcavatorState*>(&s);
        if (!st) {
            return false;
        }
        *st = resp_;
        return true;
    }

    bool setRef(const RobotState& s) override {
        std::lock_guard<std::mutex> lock(mu_);
        const auto* st = dynamic_cast<const ExcavatorState*>(&s);
        if (!st) {
            return false;
        }
        ref_ = *st;
        return true;
    }

    bool getRef(RobotState& s) const override {
        std::lock_guard<std::mutex> lock(mu_);
        auto* st = dynamic_cast<ExcavatorState*>(&s);
        if (!st) {
            return false;
        }
        *st = ref_;
        return true;
    }

private:
    mutable std::mutex mu_;
    ExcavatorControlMode mode_{};
    ExcavatorDeviceState device_state_{};
    ExcavatorState ref_{};
    ExcavatorState resp_{};
};

}  // namespace excavator
