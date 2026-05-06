#pragma once

#include <Eigen/Dense>

#include <chrono>
#include <cstdint>
#include <string>

namespace excavator_api {

using Vector8d = Eigen::Matrix<double, 8, 1>;
using Vector12i = Eigen::Matrix<int, 12, 1>;

enum class ControlMode : std::uint8_t {
    OpenLoopMotorSpeed = 0,
    ClosedLoopJointPosition = 1,
    ClosedLoopJointVelocity = 2,
    ClosedLoopVelocityScalar = 3,
};

struct SessionConfig {
    std::string can_if_name{"can2"};
    std::string imu_if_name{"can1"};
    std::string can_shm_name{"canlib_shm"};
    std::string imu_shm_name{"imu_canlib_shm"};
    bool create_mapping{false};
    bool can_simulation{true};
    bool imu_simulation{true};
    bool can_bus_enabled{true};
};

struct SpeedScalarCmd {
    Vector8d speed_scalar = Vector8d::Zero();  // [-1,1]
};

struct VelocityCmd {
    Vector8d velocity = Vector8d::Zero();
};

struct PositionCmd {
    Vector8d position = Vector8d::Zero();
};

struct RefState {
    Vector8d position = Vector8d::Zero();
    Vector8d velocity = Vector8d::Zero();
    Vector8d acceleration = Vector8d::Zero();
    Vector8d velocity_scalar = Vector8d::Zero();
    Vector12i status = Vector12i::Zero();
    Vector8d motor_rpm = Vector8d::Constant(8190.0);
    Vector8d plan_rpm = Vector8d::Constant(8190.0);
};

struct RespState {
    Vector8d position = Vector8d::Zero();
    Vector8d velocity = Vector8d::Zero();
    Vector8d acceleration = Vector8d::Zero();
    Vector8d velocity_scalar = Vector8d::Zero();
    Vector12i status = Vector12i::Zero();
    Vector8d motor_rpm = Vector8d::Constant(8190.0);
    Vector8d plan_rpm = Vector8d::Constant(8190.0);
};

struct SnapshotMeta {
    std::uint64_t loop_tick{0};
    std::uint64_t recv_time_ns{0};
};

struct Snapshot {
    RefState ref{};
    RespState resp{};
    SnapshotMeta meta{};
};

}  // namespace excavator_api
