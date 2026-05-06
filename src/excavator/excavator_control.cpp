#include <excavator/internal/excavator_control.hpp>
#include <algorithm>
#include <cmath>

namespace excavator {
namespace {
constexpr int kPidVectorCount = 8;

// 速度标量闭环前馈：|s|<=dead 不映射；外侧 (|s|-dead)/(1-dead)∈[0,1] 映射到 [|t|,1]
constexpr double kFfScalarDeadband = 0.1;
// 前馈原始转速：仅「偏离中性幅值增大」时限制每步变化；回摆靠近中性不限幅
constexpr double kFfRpmSlewFraction = 0.05;
constexpr double kFfRpmSlewNeutralEps = 1e-3;

double map_scalar_for_feedforward(double s, double threshold) {
    const double t = std::clamp(threshold, 0.0, 1.0);
    const double a = std::abs(s);
    if (a <= kFfScalarDeadband) {
        return s;
    }
    const double u = (a - kFfScalarDeadband) / (1.0 - kFfScalarDeadband);
    const double mag = (t <= 1e-15) ? u : (t + (1.0 - t) * u);
    return (s >= 0.0) ? mag : -mag;
}

double slew_feedforward_motor_rpm(double target, double prev, double d_up, double d_down) {
    const double e_prev = prev - kMotorSpeedRawZero;
    const double e_tgt = target - kMotorSpeedRawZero;
    // 靠近中性：|偏差|不增大 → 不限幅，直接跟目标
    if (std::abs(e_tgt) + kFfRpmSlewNeutralEps <= std::abs(e_prev)) {
        return std::clamp(target, kMotorSpeedRawMin, kMotorSpeedRawMax);
    }
    double delta = target - prev;
    if (delta > d_up) {
        delta = d_up;
    }
    if (delta < -d_down) {
        delta = -d_down;
    }
    return std::clamp(prev + delta, kMotorSpeedRawMin, kMotorSpeedRawMax);
}

double scalar_to_motor_rpm(double n) {
    const double scalar = std::clamp(n, -1.0, 1.0);
    const double half_span = 0.5 * (kMotorSpeedRawMax - kMotorSpeedRawMin);
    return kMotorSpeedRawZero + scalar * half_span;
}

double scalar_to_motor_rpm_by_joint(int joint_idx, double n) {
    const double scalar = (joint_idx == 2) ? (-n) : n;
    return scalar_to_motor_rpm(scalar);
}

}  // namespace

bool ExcavatorControl::setPidVectors(const std::vector<std::vector<double>>& pid_vectors) {
    if (pid_vectors.size() != kPidVectorCount) {
        return false;
    }
    PidParams params = loadPidParams();
    std::array<Vector8d*, kPidVectorCount> dst = {
        &params.position_kp,
        &params.position_ki,
        &params.position_kd,
        &params.velocity_kp,
        &params.velocity_ki,
        &params.velocity_kd,
        &params.velocity_scalar_max,
        &params.feedforward_scalar_threshold,
    };
    for (int i = 0; i < kPidVectorCount; ++i) {
        if (pid_vectors[static_cast<std::size_t>(i)].size() != kAxisCount) {
            return false;
        }
        for (int j = 0; j < kAxisCount; ++j) {
            const double v = pid_vectors[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            if (!std::isfinite(v)) {
                return false;
            }
            if (i == kPidVectorCount - 1) {
                if (v < 0.0 || v > 1.0) {
                    return false;
                }
            }
            (*dst[static_cast<std::size_t>(i)])(j) = v;
        }
    }
    std::lock_guard<std::mutex> lock(pid_mu_);
    pid_params_ = params;
    return true;
}

ExcavatorControl::PidParams ExcavatorControl::loadPidParams() const {
    std::lock_guard<std::mutex> lock(pid_mu_);
    return pid_params_;
}

void ExcavatorControl::applyZeroDriftCompensation(const ExcavatorState& resp,
                                                  const Vector8d& raw_ref_scalar,
                                                  Vector8d& compensated_resp_velocity,
                                                  Vector8d& compensated_ref_scalar) const {
    constexpr std::uint32_t kInitialBiasCycles = 20;
    if (!zero_drift_ready_) {
        resp_velocity_bias_sum_ += resp.velocity;
        ref_scalar_bias_sum_ += raw_ref_scalar;
        ++zero_drift_count_;
        if (zero_drift_count_ >= kInitialBiasCycles) {
            const double samples = static_cast<double>(zero_drift_count_);
            resp_velocity_bias_ = resp_velocity_bias_sum_ / samples;
            ref_scalar_bias_ = ref_scalar_bias_sum_ / samples;
            zero_drift_ready_ = true;
        }
    }
    compensated_resp_velocity = zero_drift_ready_ ? (resp.velocity - resp_velocity_bias_) : resp.velocity;
    compensated_ref_scalar = zero_drift_ready_ ? (raw_ref_scalar - ref_scalar_bias_) : raw_ref_scalar;
    compensated_ref_scalar = compensated_ref_scalar.array().max(-1.0).min(1.0).matrix();
}

bool ExcavatorControl::openLoopMotorSpeed(const ExcavatorState& ref_in,
                                          const ExcavatorState& resp,
                                          const ExcavatorCommand* cmd_in,
                                          ExcavatorState& ref_out) const {
    ref_out = ref_in;
    ref_out.position = resp.position;
    ref_out.velocity = resp.velocity;
    ref_out.acceleration = resp.acceleration;
    ref_out.plan_rpm.setConstant(kMotorSpeedRawZero);
    if (cmd_in) {
        ref_out.velocity_scalar = cmd_in->speed_scalar.array().max(-1.0).min(1.0).matrix();
    }
    for (int i = 0; i < kAxisCount; ++i) {
        ref_out.motor_rpm(i) = scalar_to_motor_rpm_by_joint(i, ref_out.velocity_scalar(i));
    }
    return true;
}

bool ExcavatorControl::closedLoopJointPosition(const ExcavatorState& ref_in,
                                               const ExcavatorState& resp,
                                               const ExcavatorCommand* cmd_in,
                                               ExcavatorState& ref_out) const {
    ref_out = ref_in;
    ref_out.plan_rpm.setConstant(kMotorSpeedRawZero);
    if (!cmd_in) {
        return true;
    }
    ref_out.position = cmd_in->position;
    ref_out.velocity = (ref_out.position - ref_in.position) / kTs;
    ref_out.acceleration = (ref_out.velocity - ref_in.velocity) / kTs;
    const PidParams params = loadPidParams();
    constexpr double kDeltaRpmLimit = 400.0;
    prev_ref_velocity_ = ref_out.velocity;
    for (int i = 0; i < kAxisCount; ++i) {
        const double vmax = std::max(params.velocity_scalar_max(i), 1e-6);
        ref_out.velocity_scalar(i) = std::clamp(ref_out.velocity(i) / vmax, -1.0, 1.0);
    }

    for (int i = 0; i < kAxisCount; ++i) {
        const double rpm_ff = scalar_to_motor_rpm_by_joint(i, ref_out.velocity_scalar(i));
        const double e = ref_out.position(i) - resp.position(i);
        double delta_rpm = params.position_kp(i) * (e - pid_e_prev_(i)) + params.position_ki(i) * e +
                           params.position_kd(i) * (e - 2.0 * pid_e_prev_(i) + pid_e_prev2_(i));
        delta_rpm = std::clamp(delta_rpm, -kDeltaRpmLimit, kDeltaRpmLimit);
        const double rpm_extra = pid_extra_prev_(i) + delta_rpm;
        ref_out.motor_rpm(i) = std::clamp(rpm_ff, kMotorSpeedRawMin, kMotorSpeedRawMax);
        ref_out.plan_rpm(i) = std::clamp(ref_out.motor_rpm(i) + rpm_extra, kMotorSpeedRawMin, kMotorSpeedRawMax);
        pid_e_prev2_(i) = pid_e_prev_(i);
        pid_e_prev_(i) = e;
        pid_extra_prev_(i) = rpm_extra;
    }
    return true;
}

bool ExcavatorControl::closedLoopJointVelocity(const ExcavatorState& ref_in,
                                               const ExcavatorState& resp,
                                               const ExcavatorCommand* cmd_in,
                                               ExcavatorState& ref_out) const {
    ref_out = ref_in;
    ref_out.plan_rpm.setConstant(kMotorSpeedRawZero);
    if (!cmd_in) {
        return true;
    }
    ref_out.velocity = cmd_in->velocity;
    ref_out.acceleration = (ref_out.velocity - ref_in.velocity) / kTs;
    ref_out.position = ref_in.position + ref_out.velocity * kTs;
    const PidParams params = loadPidParams();
    constexpr double kDeltaRpmLimit = 400.0;
    prev_ref_velocity_ = ref_out.velocity;
    for (int i = 0; i < kAxisCount; ++i) {
        const double vmax = std::max(params.velocity_scalar_max(i), 1e-6);
        ref_out.velocity_scalar(i) = std::clamp(ref_out.velocity(i) / vmax, -1.0, 1.0);
    }

    for (int i = 0; i < kAxisCount; ++i) {
        const double rpm_ff = scalar_to_motor_rpm_by_joint(i, ref_out.velocity_scalar(i));
        const double e_vel = ref_out.velocity(i) - resp.velocity(i);
        const double e = e_vel;
        double delta_rpm = params.velocity_kp(i) * (e - pid_e_prev_(i)) + params.velocity_ki(i) * e +
                           params.velocity_kd(i) * (e - 2.0 * pid_e_prev_(i) + pid_e_prev2_(i));
        delta_rpm = std::clamp(delta_rpm, -kDeltaRpmLimit, kDeltaRpmLimit);
        const double rpm_extra = pid_extra_prev_(i) + delta_rpm;
        ref_out.motor_rpm(i) = std::clamp(rpm_ff, kMotorSpeedRawMin, kMotorSpeedRawMax);
        ref_out.plan_rpm(i) = std::clamp(ref_out.motor_rpm(i) + rpm_extra, kMotorSpeedRawMin, kMotorSpeedRawMax);
        pid_e_prev2_(i) = pid_e_prev_(i);
        pid_e_prev_(i) = e;
        pid_extra_prev_(i) = rpm_extra;
    }
    return true;
}

bool ExcavatorControl::closedLoopVelocityScalar(const ExcavatorState& ref_in,
                                                const ExcavatorState& resp,
                                                const ExcavatorCommand* cmd_in,
                                                ExcavatorState& ref_out) const {
    ref_out = ref_in;
    ref_out.plan_rpm.setConstant(kMotorSpeedRawZero);
    if (!cmd_in) {
        return true;
    }
    const PidParams params = loadPidParams();
    constexpr double kDeltaRpmLimit = 400.0;
    Vector8d resp_velocity_c = resp.velocity;
    Vector8d ref_scalar_c = cmd_in->speed_scalar.array().max(-1.0).min(1.0).matrix();
    applyZeroDriftCompensation(resp, ref_scalar_c, resp_velocity_c, ref_scalar_c);

    ref_out.velocity_scalar = ref_scalar_c;
    const double span = kMotorSpeedRawMax - kMotorSpeedRawZero;
    const double d_up = kFfRpmSlewFraction * span;
    const double d_down = d_up;
    for (int i = 0; i < kAxisCount; ++i) {
        ref_out.velocity(i) = ref_out.velocity_scalar(i) * params.velocity_scalar_max(i);
    }
    for (int i = 0; i < kAxisCount; ++i) {
        const double s_ff = map_scalar_for_feedforward(ref_scalar_c(i), params.feedforward_scalar_threshold(i));
        const double tgt = scalar_to_motor_rpm_by_joint(i, s_ff);
        // 上一拍限幅后的前馈转速来自 ref_in.motor_rpm（通道上一帧 ref）
        ref_out.motor_rpm(i) = slew_feedforward_motor_rpm(tgt, ref_in.motor_rpm(i), d_up, d_down);
    }
    ref_out.acceleration = (ref_out.velocity - ref_in.velocity) / kTs;
    ref_out.position = ref_in.position + ref_out.velocity * kTs;
    prev_ref_velocity_ = ref_out.velocity;

    for (int i = 0; i < kAxisCount; ++i) {
        const double e = ref_out.velocity(i) - resp_velocity_c(i);
        double delta_rpm = params.velocity_kp(i) * (e - pid_e_prev_(i)) + params.velocity_ki(i) * e +
                           params.velocity_kd(i) * (e - 2.0 * pid_e_prev_(i) + pid_e_prev2_(i));
        delta_rpm = std::clamp(delta_rpm, -kDeltaRpmLimit, kDeltaRpmLimit);
        const double rpm_extra = pid_extra_prev_(i) + delta_rpm;
        ref_out.plan_rpm(i) = std::clamp(ref_out.motor_rpm(i) + rpm_extra, kMotorSpeedRawMin, kMotorSpeedRawMax);
        pid_e_prev2_(i) = pid_e_prev_(i);
        pid_e_prev_(i) = e;
        pid_extra_prev_(i) = rpm_extra;
    }
    return true;
}

bool ExcavatorControl::updateRef(const ExcavatorState& ref_in,
                                 const ExcavatorState& resp,
                                 const ExcavatorCommand* cmd_in,
                                 ExcavatorState& ref_out) const {
    ExcavatorControlMode mode{};
    if (!state_.getControlMode(mode)) {
        ref_out = ref_in;
        return false;
    }
    if (mode.mode != last_mode_) {
        last_mode_ = mode.mode;
        mode_cycle_ = 0;
        prev_ref_velocity_.setZero();
        pid_e_prev_.setZero();
        pid_e_prev2_.setZero();
        pid_extra_prev_.setZero();
        pid_rpm_prev_.setConstant(kMotorSpeedRawZero);
        resp_velocity_bias_sum_.setZero();
        resp_velocity_bias_.setZero();
        ref_scalar_bias_sum_.setZero();
        ref_scalar_bias_.setZero();
        zero_drift_count_ = 0;
        zero_drift_ready_ = false;
    }
    switch (mode.mode) {
        case ExcavatorControlModeType::OpenLoopMotorSpeed:
            return openLoopMotorSpeed(ref_in, resp, cmd_in, ref_out);
        case ExcavatorControlModeType::ClosedLoopJointPosition:
            return closedLoopJointPosition(ref_in, resp, cmd_in, ref_out);
        case ExcavatorControlModeType::ClosedLoopJointVelocity:
            return closedLoopJointVelocity(ref_in, resp, cmd_in, ref_out);
        case ExcavatorControlModeType::ClosedLoopVelocityScalar:
            return closedLoopVelocityScalar(ref_in, resp, cmd_in, ref_out);
        default:
            ref_out = ref_in;
            return false;
    }
}

}  // namespace excavator
