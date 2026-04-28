#pragma once

#include <excavator/internal/excavator_data_type.hpp>
#include <hal/communication_interface.hpp>

#include <memory>
#include <string>

namespace excavator {

class ExcavatorCommunication final : public CommunicationInterface {
public:
    explicit ExcavatorCommunication(std::string can_mapping_name, bool create_mapping = false,
                                  std::string imu_mapping_name = "imu_canlib_shm");
    ~ExcavatorCommunication() override;

    ExcavatorCommunication(const ExcavatorCommunication&) = delete;
    ExcavatorCommunication& operator=(const ExcavatorCommunication&) = delete;

    bool open() override;
    bool close() override;
    bool read() override;
    bool write() override;
    bool isOpen() const override;

    /** 最近一次 read() 写入的反馈缓冲 */
    ExcavatorHardwareState& mutableHardwareState() noexcept { return hw_state_; }
    const ExcavatorHardwareState& hardwareState() const noexcept { return hw_state_; }

    /** write() 时从该缓冲序列化到 SHM（对 canlib 命名的 HardwareCommand） */
    ExcavatorHardwareCommand& mutableHardwareCommand() noexcept { return hw_cmd_; }
    const ExcavatorHardwareCommand& hardwareCommand() const noexcept { return hw_cmd_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    ExcavatorHardwareState hw_state_{};
    ExcavatorHardwareCommand hw_cmd_{};
    bool has_valid_imu_{false};
};

}  // namespace excavator
