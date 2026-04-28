#pragma once

#include <Eigen/Geometry>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace canlib {

inline constexpr std::uint8_t kImuDeviceCount = 4U;
inline constexpr std::uint16_t kImuBaseIdHighSpeedCh1 = 0x200U;
inline constexpr std::size_t kImuCanPayloadBytes = 8U;

// 单设备分包累加状态（高速 ch1 解析用）
struct ImuRxAccumulator {
    bool has_euler{false};
    bool has_gyro{false};
    bool has_accel{false};
    bool has_quat_1{false};
    bool has_quat_2{false};
    bool has_status{false};

    float roll_rad{0.0F};
    float pitch_rad{0.0F};
    float yaw_rad{0.0F};
    float gyro_x_dps{0.0F};
    float gyro_y_dps{0.0F};
    float gyro_z_dps{0.0F};
    float accel_x_mps2{0.0F};
    float accel_y_mps2{0.0F};
    float accel_z_mps2{0.0F};

    float q0{1.0F};
    float q1{0.0F};
    float q2{0.0F};
    float q3{0.0F};

    std::uint32_t timestamp_ms{0};
    std::uint8_t valid_flags{0};
    std::uint64_t last_rx_ns{0};
};

// 预留：替换 parse_frame；默认实现与当前协议一致
class ImuCanFrameParser {
public:
    virtual ~ImuCanFrameParser() = default;
    virtual void parseFrame(std::uint16_t can_id, const std::array<std::uint8_t, kImuCanPayloadBytes>& payload,
                            std::array<ImuRxAccumulator, kImuDeviceCount>& partials) = 0;
};

class ImuDefaultCanFrameParser final : public ImuCanFrameParser {
public:
    void parseFrame(std::uint16_t can_id, const std::array<std::uint8_t, kImuCanPayloadBytes>& payload,
                    std::array<ImuRxAccumulator, kImuDeviceCount>& partials) final;
};

struct ImuSample {
    std::uint8_t device_addr{0};
    std::uint8_t online{0};
    std::uint8_t valid_attitude{0};
    std::uint8_t valid_gyro{0};
    std::uint8_t valid_accel{0};
    std::uint8_t reserved0{0};
    std::uint16_t packet_loss_count{0};
    std::uint32_t imu_timestamp_ms{0};
    std::uint64_t host_rx_time_ns{0};

    Eigen::Vector3f rpy_rad{Eigen::Vector3f::Zero()};    // roll,pitch,yaw
    Eigen::Vector3f gyro_dps{Eigen::Vector3f::Zero()};   // x,y,z
    Eigen::Vector3f accel_mps2{Eigen::Vector3f::Zero()}; // x,y,z

    Eigen::Quaternionf quaternion{1.0F, 0.0F, 0.0F, 0.0F};  // w,x,y,z
};

struct ImuSharedMemoryLayout {
    std::uint64_t magic{0x494D555F43414E31ULL};
    std::uint64_t sequence{0};
    std::array<ImuSample, kImuDeviceCount> imus{};
};

class ImuCanLib {
public:
    // 默认 can1 + 模拟；真实总线请在 open() 前 setSimulationEnabled(false)
    ImuCanLib(std::string can_if_name = "can1", std::string shm_name = "imu_canlib_shm", bool create_mapping = false);
    ~ImuCanLib();

    ImuCanLib(const ImuCanLib&) = delete;
    ImuCanLib& operator=(const ImuCanLib&) = delete;

    bool open();
    bool start();
    bool stop();
    bool close();
    bool isOpen() const;
    std::string lastError() const;
    std::uint64_t loopTick() const;
    // 须在 open() 之前置 true：不打开 CAN，50Hz 仅向共享内存写初始姿态
    void setSimulationEnabled(bool enabled);
    bool isSimulationEnabled() const;
    // nullptr 使用 ImuDefaultCanFrameParser；须在 start() 前调用
    void setFrameParser(std::unique_ptr<ImuCanFrameParser> parser);
    ImuCanFrameParser* frameParser();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace canlib
