#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <hal/data_types.hpp>

#include <array>
#include <cstdint>
#include <memory>

namespace excavator {

inline constexpr int kAxisCount = 8; // 13-20
inline constexpr int kStatusCount = 12;   // 1-12
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kHalfPi = kPi / 2.0;
inline constexpr double kHz = 50.0;
inline constexpr double kTs = 1.0 / kHz;

using Vector8d = Eigen::Matrix<double, kAxisCount, 1>;
using Vector12i = Eigen::Matrix<int, kStatusCount, 1>;

// 控制模式：开环电机转速 / 闭环关节位置 / 闭环关节速度
enum class ExcavatorControlModeType : std::uint8_t {
    OpenLoopMotorSpeed = 0,
    ClosedLoopJointPosition = 1,
    ClosedLoopJointVelocity = 2,
    ClosedLoopVelocityScalar = 3,
};

inline constexpr std::size_t kImuDeviceCount = 4;

class ExcavatorDeviceState final : public DeviceState {
public:
    std::uint32_t fault_code{0};
    std::unique_ptr<AbstractType> clone() const override {
        return std::make_unique<ExcavatorDeviceState>(*this);
    }
};

class ExcavatorControlMode final : public ::ControlMode {
public:
    ExcavatorControlModeType mode{ExcavatorControlModeType::OpenLoopMotorSpeed};
    std::unique_ptr<AbstractType> clone() const override {
        return std::make_unique<ExcavatorControlMode>(*this);
    }
};

struct ExcavatorImuHardwareState {
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
        Eigen::Vector3f rpy_rad{Eigen::Vector3f::Zero()};
        Eigen::Vector3f gyro_dps{Eigen::Vector3f::Zero()};
        Eigen::Vector3f accel_mps2{Eigen::Vector3f::Zero()};
        Eigen::Quaternionf quaternion{1.0F, 0.0F, 0.0F, 0.0F};
    };
    std::array<ImuSample, kImuDeviceCount> devices{};
};

inline constexpr double kMotorSpeedRawMin = 1638.0;
inline constexpr double kMotorSpeedRawZero = 8190.0;
inline constexpr double kMotorSpeedRawMax = 14742.0;

struct ExcavatorMotorHardwareState {
    Vector12i status = Vector12i::Zero();  // 点火/遥控等离散参考位
    Vector8d motor_rpm = Vector8d::Constant(kMotorSpeedRawZero);
};

// 融合状态：通道 ref/resp；IMU 原始样本不入此结构，由 Client 从 SHM 直读并写入 position/velocity 等
class ExcavatorState final : public RobotState {
public:
    Vector8d position = Vector8d::Zero();
    Vector8d velocity = Vector8d::Zero();
    Vector8d acceleration = Vector8d::Zero();
    Vector8d velocity_scalar = Vector8d::Zero();
    Vector12i status = Vector12i::Zero();
    Vector8d motor_rpm = Vector8d::Constant(kMotorSpeedRawZero);
    Vector8d plan_rpm = Vector8d::Constant(kMotorSpeedRawZero);

    std::unique_ptr<AbstractType> clone() const override {
        return std::make_unique<ExcavatorState>(*this);
    }
};

// 硬件聚合：SHM 关节/状态 + motor；imu 供与 canlib 对齐的占位（可由上层填入再 hardwareToState）
class ExcavatorHardwareState final : public HardwareState {
public:
    ExcavatorMotorHardwareState motor{};
    ExcavatorImuHardwareState imu{};

    std::unique_ptr<AbstractType> clone() const override {
        return std::make_unique<ExcavatorHardwareState>(*this);
    }
};

// 外部→上层：RobotCommand；经 Converter 入 RobotState（motor_rpm 全中性时用 velocity 填电机语义）
class ExcavatorCommand final : public RobotCommand {
public:
    Vector8d position = Vector8d::Zero();
    Vector8d velocity = Vector8d::Zero();
    Vector8d speed_scalar = Vector8d::Zero();  // [-1,1]
    Vector8d motor_rpm = Vector8d::Constant(kMotorSpeedRawZero);

    std::unique_ptr<AbstractType> clone() const override {
        return std::make_unique<ExcavatorCommand>(*this);
    }
};

// 上层→通信层：仅保留抽象硬件命令语义
class ExcavatorHardwareCommand final : public HardwareCommand {
public:
    Vector12i status = Vector12i::Zero();
    Vector8d motor_rpm = Vector8d::Constant(kMotorSpeedRawZero);

    std::unique_ptr<AbstractType> clone() const override {
        return std::make_unique<ExcavatorHardwareCommand>(*this);
    }
};

}  // namespace excavator
