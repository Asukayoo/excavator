#include <excavator/internal/excavator_converter.hpp>

namespace excavator {
namespace {

double dps_to_radps(double dps) noexcept { return dps * kPi / 180.0; }

/** 交换前4关节的2/3号位（1234 <-> 1324），其余轴保持不变。 */
Vector8d swap_joint_2_3_on_first4(const Vector8d& v) noexcept {
    Vector8d out = v;
    out(1) = v(2);
    out(2) = v(1);
    return out;
}

void fill_kinematic_from_imu_hw(const ExcavatorHardwareState& hw, ExcavatorState& st) noexcept {
    st.position.setZero();
    st.velocity.setZero();
    st.acceleration.setZero();
    // 关节1234 <- imu4321，轴向为 z y y y。
    const auto& imu1 = hw.imu.devices[0];
    const auto& imu2 = hw.imu.devices[1];
    const auto& imu3 = hw.imu.devices[2];
    const auto& imu4 = hw.imu.devices[3];

    st.position(0) = static_cast<double>(imu4.rpy_rad(2));
    st.velocity(0) = dps_to_radps(static_cast<double>(imu4.gyro_dps(2)));
    st.acceleration(0) = static_cast<double>(imu4.accel_mps2(2));

    st.position(1) = static_cast<double>(imu3.rpy_rad(1));
    st.velocity(1) = dps_to_radps(static_cast<double>(imu3.gyro_dps(1)));
    st.acceleration(1) = static_cast<double>(imu3.accel_mps2(1));

    st.position(2) = static_cast<double>(imu2.rpy_rad(1));
    st.velocity(2) = dps_to_radps(static_cast<double>(imu2.gyro_dps(1)));
    st.acceleration(2) = static_cast<double>(imu2.accel_mps2(1));

    st.position(3) = static_cast<double>(imu1.rpy_rad(1));
    st.velocity(3) = dps_to_radps(static_cast<double>(imu1.gyro_dps(1)));
    st.acceleration(3) = static_cast<double>(imu1.accel_mps2(1));
}

}  // namespace

bool ExcavatorConverter::robotCmdToRobotState(const RobotCommand& cmd, RobotState& state_out) {
    const auto* c = asCommand(cmd);
    auto* r = asState(state_out);
    if (!c || !r) {
        return false;
    }
    r->position = c->position;
    r->velocity = c->velocity;
    const double off =
        (c->motor_rpm.array() - kMotorSpeedRawZero).matrix().template lpNorm<Eigen::Infinity>();
    r->motor_rpm = (off < 1e-6) ? c->velocity : c->motor_rpm;
    return true;
}

bool ExcavatorConverter::robotStateToHardwareCmd(const RobotState& state, HardwareCommand& cmd_out) {
    const auto* st = asState(state);
    auto* hw = asHardwareCmd(cmd_out);
    if (!st || !hw) {
        return false;
    }
    hw->status = st->status;
    hw->motor_rpm = swap_joint_2_3_on_first4(st->motor_rpm);
    return true;
}

bool ExcavatorConverter::hardwareStateToRobotState(const HardwareState& raw_in, RobotState& state_out) {
    const auto* hw = asHardwareState(raw_in);
    auto* st = asState(state_out);
    if (!hw || !st) {
        return false;
    }
    st->status = hw->motor.status;
    st->motor_rpm = swap_joint_2_3_on_first4(hw->motor.motor_rpm);
    fill_kinematic_from_imu_hw(*hw, *st);
    // 差分语义：J3=J3-J2，J4=J4-J3（使用变换前值避免串扰）。
    const double theta2_raw = st->position(1);
    const double theta3_raw = st->position(2);
    const double theta4_raw = st->position(3);
    st->position(2) = theta3_raw - theta2_raw;
    st->position(3) = theta4_raw - theta3_raw;

    const double omega2_raw = st->velocity(1);
    const double omega3_raw = st->velocity(2);
    const double omega4_raw = st->velocity(3);
    st->velocity(2) = omega3_raw - omega2_raw;
    st->velocity(3) = omega4_raw - omega3_raw;
    constexpr std::uint32_t kInitialBiasCycles = 20;
    if (!resp_velocity_bias_ready_) {
        resp_velocity_bias_sum_ += st->velocity;
        ++resp_velocity_bias_count_;
        if (resp_velocity_bias_count_ >= kInitialBiasCycles) {
            resp_velocity_bias_ = resp_velocity_bias_sum_ / static_cast<double>(resp_velocity_bias_count_);
            resp_velocity_bias_ready_ = true;
        }
    }
    if (resp_velocity_bias_ready_) {
        st->velocity -= resp_velocity_bias_;
    }
    return true;
}

}  // namespace excavator
