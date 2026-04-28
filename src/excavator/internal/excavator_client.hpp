#pragma once

#include <excavator/internal/excavator_communication.hpp>
#include <excavator/internal/excavator_control.hpp>
#include <excavator/internal/excavator_converter.hpp>
#include <excavator/internal/excavator_data_type.hpp>
#include <excavator/internal/excavator_state_channel.hpp>
#include <hal/driver_interface.hpp>

#include <atomic>
#include <array>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace excavator {

class ExcavatorClient : public DriverInterface {
public:
    explicit ExcavatorClient(std::string can_shm_name = "canlib_shm",
                             bool create_mapping = false,
                             std::string imu_shm_name = "imu_canlib_shm");
    ~ExcavatorClient() override;

    bool init() override;
    bool start() override;
    bool stop() override;
    bool reset() override;

    /** 外部伺服指令入队 */
    void submitServo(const ExcavatorCommand& cmd);
    bool setPidVectors(const std::vector<std::vector<double>>& pid_vectors);
    void clearServo();
    // 状态切换接口（点火~急停，索引 0~10）。
    void toggleIgnition();
    void toggleFlameout();
    void toggleCrush();
    void toggleChassisLight();
    void toggleRemoteMode();
    void togglePilot();
    void toggleHighSpeed();
    void toggleChassisDozerMode();
    void toggleHorn();
    void toggleMotorGear();
    void toggleEstop();
    // 批量切换：bit0..10 分别对应点火~急停。
    void applyStatusToggleMask(std::uint16_t toggle_mask);
    std::uint64_t loopTick() const noexcept { return loop_tick_.load(std::memory_order_acquire); }

    StateInterface* mutableState() override { return &channel_; }
    const StateInterface* state() const override { return &channel_; }
    const DeviceState& getDeviceState() const override { return device_; }

protected:
    void loop() override;
    bool update() override;

private:
    bool takeServoCmd(ExcavatorCommand& out_cmd);
    void toggleStatusBit(int status_idx);
    void applyCachedStatusToRef(ExcavatorState& ref);

    ExcavatorCommunication comm_;
    ExcavatorControl controller_;
    ExcavatorConverter converter_{};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> loop_tick_{0};
    std::thread worker_{};
    ExcavatorStateChannel channel_{};
    ExcavatorDeviceState device_{};
    std::deque<ExcavatorCommand> ref_command_queue_{};
    mutable std::mutex ref_command_queue_mu_{};
    Vector12i cached_status_{Vector12i::Zero()};
    mutable std::mutex cached_status_mu_{};
};

}  // namespace excavator
